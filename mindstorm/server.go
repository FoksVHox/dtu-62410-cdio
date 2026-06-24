package mindstorm

import (
	"bufio"
	"bot/config"
	"fmt"
	"net"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/apex/log"
)

// CommandServer listens for drive commands streamed from the PC (the vision /
// navigation program) and applies them to the belt drive.
//
// Wire protocol (one command per line, newline-terminated):
//
//	"<throttle> <turn>\n"   e.g. "0.500 -0.200\n"  (values in [-1, 1])
//	"STOP\n"                halt the belts
//	"LATCH_OPEN\n"          fire the back motor to open the ball-release latch
//
// A watchdog stops the belts automatically if no command is received within
// CommandTimeout, so the robot never runs away when the link drops.
type CommandServer struct {
	drive   *BeltDrive
	back    *Motor
	addr    string
	timeout time.Duration

	mu       sync.Mutex
	lastSeen time.Time
	moving   bool
}

// NewCommandServer creates a server that drives `drive` from commands received
// on `addr` (for example ":9000"). The `back` motor is used to open the
// ball-release latch when the LATCH_OPEN command is received.
func NewCommandServer(drive *BeltDrive, back *Motor, addr string) *CommandServer {
	if strings.TrimSpace(addr) == "" {
		addr = ":9000"
	}
	return &CommandServer{
		drive:   drive,
		back:    back,
		addr:    addr,
		timeout: 750 * time.Millisecond,
	}
}

// ListenAndServe blocks, accepting one PC client at a time and applying its
// drive commands until the process is stopped.
func (s *CommandServer) ListenAndServe() error {
	ln, err := net.Listen("tcp", s.addr)
	if err != nil {
		return fmt.Errorf("mindstorm: listen on %q: %w", s.addr, err)
	}
	defer ln.Close()

	log.WithField("address", s.addr).Info("mindstorm: drive command server listening")

	// Safety watchdog: stop the belts if commands go stale.
	go s.runWatchdog()

	for {
		conn, err := ln.Accept()
		if err != nil {
			log.WithError(err).Error("mindstorm: accept failed")
			continue
		}
		log.WithField("remote", conn.RemoteAddr().String()).Info("mindstorm: PC connected")
		s.handleConn(conn)
		log.Info("mindstorm: PC disconnected, stopping belts")
		s.stop()
	}
}

func (s *CommandServer) handleConn(conn net.Conn) {
	defer conn.Close()

	// Seed the watchdog so it doesn't trip the instant a client connects.
	s.mark()

	scanner := bufio.NewScanner(conn)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		s.applyLine(line)
	}
	if err := scanner.Err(); err != nil {
		log.WithError(err).Warn("mindstorm: connection read error")
	}
}

// applyLine parses and applies a single protocol line.
func (s *CommandServer) applyLine(line string) {
	s.mark()

	if strings.EqualFold(line, "STOP") {
		s.stop()
		return
	}

	// LATCH_OPEN: stop the belts then fire the back motor to release balls.
	if strings.EqualFold(line, "LATCH_OPEN") {
		s.openLatch()
		return
	}

	fields := strings.Fields(line)
	if len(fields) != 2 {
		log.WithField("line", line).Warn("mindstorm: malformed command (expected '<throttle> <turn>')")
		return
	}

	throttle, err1 := strconv.ParseFloat(fields[0], 64)
	turn, err2 := strconv.ParseFloat(fields[1], 64)
	if err1 != nil || err2 != nil {
		log.WithField("line", line).Warn("mindstorm: could not parse command values")
		return
	}

	if err := s.drive.SetThrottle(throttle, turn); err != nil {
		log.WithError(err).Error("mindstorm: failed to apply drive command")
		return
	}

	s.mu.Lock()
	s.moving = true
	s.mu.Unlock()
}

// openLatch stops the belts and activates the back motor for the configured
// latch-open duration so the harvested balls are released into the goal.
func (s *CommandServer) openLatch() {
	log.Info("mindstorm: LATCH_OPEN received — stopping belts and opening latch")
	s.stop()

	if s.back == nil {
		log.Warn("mindstorm: back motor not configured, cannot open latch")
		return
	}

	motorCfg := config.Get().Mindstorm.Motors
	backSpeed := int(float64(s.back.MaxSpeedTPS()) * motorCfg.Back.Speed)
	latchDurationMS := int(motorCfg.LatchOpenTime.Milliseconds())

	log.WithFields(log.Fields{
		"speed_tps":    backSpeed,
		"duration_ms":  latchDurationMS,
	}).Info("mindstorm: firing back motor (latch open)")

	if err := s.back.RunTimed(backSpeed, latchDurationMS); err != nil {
		log.WithError(err).Error("mindstorm: failed to fire back motor for latch open")
	}
}

func (s *CommandServer) stop() {
	if err := s.drive.Stop(); err != nil {
		log.WithError(err).Error("mindstorm: failed to stop belts")
	}
	s.mu.Lock()
	s.moving = false
	s.mu.Unlock()
}

func (s *CommandServer) mark() {
	s.mu.Lock()
	s.lastSeen = time.Now()
	s.mu.Unlock()
}

// runWatchdog stops the belts if no command has arrived within the timeout.
func (s *CommandServer) runWatchdog() {
	ticker := time.NewTicker(s.timeout / 2)
	defer ticker.Stop()

	for range ticker.C {
		s.mu.Lock()
		stale := s.moving && time.Since(s.lastSeen) > s.timeout
		s.mu.Unlock()

		if stale {
			log.Warn("mindstorm: command timeout, stopping belts (watchdog)")
			s.stop()
		}
	}
}
