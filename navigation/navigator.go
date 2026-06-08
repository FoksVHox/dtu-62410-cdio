// Package navigation implements the high-level navigation state machine for the
// golfbot.  It reads ball detections from the vision package and translates them
// into BeltDrive commands sent to the LEGO Mindstorms EV3.
package navigation

import (
	"context"
	"fmt"
	"math"
	"time"

	"bot/mindstorm"
	"bot/vision"

	"github.com/apex/log"
	gocv "gocv.io/x/gocv"
)

// State represents the navigation state machine states.
type State int

const (
	// StateSearching — no ball visible; robot rotates to scan the environment.
	StateSearching State = iota
	// StateApproaching — ball detected; robot drives toward it.
	StateApproaching
	// StateCollecting — ball is close enough; robot activates collector then returns to search.
	StateCollecting
)

func (s State) String() string {
	switch s {
	case StateSearching:
		return "searching"
	case StateApproaching:
		return "approaching"
	case StateCollecting:
		return "collecting"
	default:
		return "unknown"
	}
}

// Config holds tunable navigation parameters.
type Config struct {
	// DriveSpeed is the base forward throttle in [0, 1].
	DriveSpeed float64
	// TurnSpeed is the rotation throttle used while searching.
	TurnSpeed float64
	// SteeringGain scales the horizontal ball offset into a turn correction.
	SteeringGain float64
	// CollectDistanceThreshold: when EstimatedDistance drops below this value
	// the robot transitions to StateCollecting.
	CollectDistanceThreshold float64
	// DistanceK passed to Ball.EstimatedDistance.
	DistanceK float64
	// SearchTimeout: after this long without seeing a ball the robot turns.
	SearchTimeout time.Duration
	// CollectDwellTime: how long the collector motor runs during pickup.
	CollectDwellTime time.Duration
	// TickInterval controls how fast the main loop runs.
	TickInterval time.Duration
}

// DefaultConfig returns conservative defaults for a first test.
func DefaultConfig() Config {
	return Config{
		DriveSpeed:               0.35,
		TurnSpeed:                0.25,
		SteeringGain:             0.6,
		CollectDistanceThreshold: 1.5, // tune on real robot
		DistanceK:                vision.DetectorDefaultDistanceK,
		SearchTimeout:            2 * time.Second,
		CollectDwellTime:         1200 * time.Millisecond,
		TickInterval:             50 * time.Millisecond,
	}
}

// Navigator is the top-level controller that ties vision and motor control together.
type Navigator struct {
	cfg      Config
	drive    *mindstorm.BeltDrive
	collect  *mindstorm.Motor // the back/collector motor
	cam      *vision.Camera
	detector *vision.Detector

	state        State
	lastSeen     time.Time
	collectStart time.Time

	// frameWidth / frameHeight are cached from the camera after the first frame.
	frameWidth  int
	frameHeight int
}

// New creates a Navigator.  The collect motor may be nil if there is no separate
// collector mechanism.
func New(
	cfg Config,
	drive *mindstorm.BeltDrive,
	collect *mindstorm.Motor,
	cam *vision.Camera,
	detector *vision.Detector,
) *Navigator {
	return &Navigator{
		cfg:      cfg,
		drive:    drive,
		collect:  collect,
		cam:      cam,
		detector: detector,
		state:    StateSearching,
		lastSeen: time.Now(),
	}
}

// Run starts the navigation loop and blocks until ctx is cancelled or a fatal
// error occurs.  It is safe to call Stop() from another goroutine while Run is
// executing.
func (n *Navigator) Run(ctx context.Context) error {
	frame := gocv.NewMat()
	defer frame.Close()

	ticker := time.NewTicker(n.cfg.TickInterval)
	defer ticker.Stop()

	log.WithField("state", n.state).Info("navigation: starting loop")

	for {
		select {
		case <-ctx.Done():
			log.Info("navigation: context cancelled, stopping")
			_ = n.drive.Stop()
			return ctx.Err()
		case <-ticker.C:
			if err := n.tick(&frame); err != nil {
				log.WithError(err).Error("navigation: tick error")
			}
		}
	}
}

// tick executes one control loop iteration.
func (n *Navigator) tick(frame *gocv.Mat) error {
	// Grab frame.
	if err := n.cam.Read(frame); err != nil {
		return fmt.Errorf("navigation: read frame: %w", err)
	}

	// Cache frame dimensions.
	if n.frameWidth == 0 {
		n.frameWidth = frame.Cols()
		n.frameHeight = frame.Rows()
	}

	// Detect balls.
	balls, err := n.detector.Detect(*frame)
	if err != nil {
		return fmt.Errorf("navigation: detection: %w", err)
	}

	switch n.state {
	case StateSearching:
		return n.handleSearching(balls)
	case StateApproaching:
		return n.handleApproaching(balls)
	case StateCollecting:
		return n.handleCollecting()
	}
	return nil
}

// handleSearching rotates until a ball is found.
func (n *Navigator) handleSearching(balls []vision.Ball) error {
	if len(balls) > 0 {
		log.WithFields(log.Fields{
			"ball_x": balls[0].Center.X,
			"ball_y": balls[0].Center.Y,
		}).Info("navigation: ball found, switching to approaching")
		n.lastSeen = time.Now()
		n.transitionTo(StateApproaching)
		return nil
	}

	// Keep rotating to scan.
	return n.drive.Turn(n.cfg.TurnSpeed)
}

// handleApproaching steers the robot toward the closest detected ball.
func (n *Navigator) handleApproaching(balls []vision.Ball) error {
	if len(balls) == 0 {
		// Lost sight of ball.
		if time.Since(n.lastSeen) > n.cfg.SearchTimeout {
			log.Info("navigation: ball lost, returning to searching")
			n.transitionTo(StateSearching)
		}
		// Keep last command active for a moment.
		return nil
	}

	n.lastSeen = time.Now()
	target := balls[0] // largest = closest

	dist := target.EstimatedDistance(n.cfg.DistanceK)

	log.WithFields(log.Fields{
		"dist":   fmt.Sprintf("%.2f", dist),
		"radius": fmt.Sprintf("%.1f", target.Radius),
		"norm_x": fmt.Sprintf("%.3f", target.NormX(n.frameWidth)),
	}).Debug("navigation: approaching ball")

	// Close enough — collect!
	if dist <= n.cfg.CollectDistanceThreshold {
		log.Info("navigation: ball in range, switching to collecting")
		n.transitionTo(StateCollecting)
		return nil
	}

	// Proportional steering: offset in [-1,1] multiplied by gain gives turn correction.
	normX := target.NormX(n.frameWidth)
	turnCorrection := clamp(normX*n.cfg.SteeringGain, -1, 1)

	// Scale drive speed down when heavily off-axis to avoid overshooting.
	speedScale := 1.0 - math.Abs(normX)*0.4
	throttle := n.cfg.DriveSpeed * speedScale

	return n.drive.SetThrottle(throttle, turnCorrection)
}

// handleCollecting activates the collector motor and waits for the dwell time.
func (n *Navigator) handleCollecting() error {
	if n.collectStart.IsZero() {
		// First tick in this state: stop driving, start collector.
		if err := n.drive.Stop(); err != nil {
			return err
		}
		if n.collect != nil {
			// Run collector for the dwell duration.
			if err := n.collect.RunTimed(600, int(n.cfg.CollectDwellTime.Milliseconds())); err != nil {
				log.WithError(err).Warn("navigation: collector motor error")
			}
		}
		n.collectStart = time.Now()
		log.Info("navigation: collecting ball")
		return nil
	}

	// Wait until dwell time has elapsed.
	if time.Since(n.collectStart) >= n.cfg.CollectDwellTime {
		log.Info("navigation: collection done, returning to searching")
		n.collectStart = time.Time{}
		n.transitionTo(StateSearching)
	}
	return nil
}

func (n *Navigator) transitionTo(s State) {
	log.WithFields(log.Fields{"from": n.state, "to": s}).Info("navigation: state transition")
	n.state = s
}

// Stop immediately halts the drive motors.  Safe to call from any goroutine.
func (n *Navigator) Stop() error {
	return n.drive.Stop()
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
