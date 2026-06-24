package main

import (
	"fmt"
	"math"
	"net"
	"sync"
	"time"
)

// RobotLink sends DriveCommands to the physical robot.
//
// The vision program runs on a laptop while the motors live on the EV3, so we
// stream commands over a tiny TCP channel. Each command is a single line:
//
//	"<throttle> <turn>\n"     e.g. "0.50 -0.20\n"
//	"STOP\n"                  to halt the belts
//
// The EV3 side listens on the same port and maps throttle/turn onto the belt
// drive. If no address is configured (or the connection drops), RobotLink
// degrades gracefully and only logs to stdout so the vision UI keeps running.
type RobotLink struct {
	mu       sync.Mutex
	conn     net.Conn
	addr     string
	enabled  bool
	lastSend time.Time

	// Smoothing: limit how fast throttle/turn can change per command to avoid
	// jerky motion and wheel slip.
	maxStep  float64
	curThr   float64
	curTurn  float64
	minPause time.Duration
}

// NewRobotLink creates a link to the robot. Pass an empty addr to run the
// vision program in "simulation" mode (commands are printed, not sent).
func NewRobotLink(addr string) *RobotLink {
	rl := &RobotLink{
		addr:     addr,
		enabled:  addr != "",
		maxStep:  0.15,
		minPause: 80 * time.Millisecond,
	}
	if rl.enabled {
		rl.dialWithRetry()
	}
	return rl
}

// dialWithRetry keeps trying to connect to the robot until it succeeds.
// It blocks the caller, so it should only be called from NewRobotLink (at
// startup) or from a goroutine when reconnecting after a dropped connection.
func (rl *RobotLink) dialWithRetry() {
	const retryInterval = 2 * time.Second
	attempt := 0
	for {
		attempt++
		conn, err := net.DialTimeout("tcp", rl.addr, 2*time.Second)
		if err == nil {
			fmt.Printf("[RobotLink] connected to robot at %s (attempt %d)\n", rl.addr, attempt)
			rl.conn = conn
			return
		}
		fmt.Printf("[RobotLink] could not connect to %s (attempt %d): %v — retrying in %s\n",
			rl.addr, attempt, err, retryInterval)
		time.Sleep(retryInterval)
	}
}

// reconnectAsync closes the current (dead) connection and spawns a background
// goroutine that retries until the robot is reachable again.
func (rl *RobotLink) reconnectAsync() {
	if rl.conn != nil {
		_ = rl.conn.Close()
		rl.conn = nil
	}
	go func() {
		rl.mu.Lock()
		rl.dialWithRetry()
		rl.mu.Unlock()
	}()
}

// Send applies smoothing/rate-limiting and forwards the command to the robot.
func (rl *RobotLink) Send(cmd DriveCommand) {
	rl.mu.Lock()
	defer rl.mu.Unlock()

	// Rate limit so we don't flood the EV3.
	if time.Since(rl.lastSend) < rl.minPause {
		return
	}
	rl.lastSend = time.Now()

	targetThr, targetTurn := cmd.Throttle, cmd.Turn
	if cmd.Arrived {
		targetThr, targetTurn = 0, 0
	}

	// Smoothly ramp current values toward the target.
	rl.curThr = approach(rl.curThr, targetThr, rl.maxStep)
	rl.curTurn = approach(rl.curTurn, targetTurn, rl.maxStep)

	line := fmt.Sprintf("%.3f %.3f\n", clamp(rl.curThr, -1, 1), clamp(rl.curTurn, -1, 1))
	if cmd.Arrived && rl.curThr == 0 && rl.curTurn == 0 {
		line = "STOP\n"
	}

	if !rl.enabled || rl.conn == nil {
		// Simulation mode or still reconnecting: just show what we would send.
		fmt.Printf("[RobotLink:SIM] %s", line)
		return
	}

	if _, err := rl.conn.Write([]byte(line)); err != nil {
		fmt.Printf("[RobotLink] write failed: %v (reconnecting)\n", err)
		rl.reconnectAsync()
	}
}

// Stop tells the robot to halt and resets the smoothing state.
func (rl *RobotLink) Stop() {
	rl.mu.Lock()
	defer rl.mu.Unlock()
	rl.curThr, rl.curTurn = 0, 0
	if rl.conn != nil {
		_, _ = rl.conn.Write([]byte("STOP\n"))
	} else {
		fmt.Println("[RobotLink:SIM] STOP")
	}
}

// Close stops the robot and closes the connection.
func (rl *RobotLink) Close() {
	rl.Stop()
	rl.mu.Lock()
	defer rl.mu.Unlock()
	if rl.conn != nil {
		_ = rl.conn.Close()
		rl.conn = nil
	}
}

func approach(cur, target, step float64) float64 {
	if math.Abs(target-cur) <= step {
		return target
	}
	if target > cur {
		return cur + step
	}
	return cur - step
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
