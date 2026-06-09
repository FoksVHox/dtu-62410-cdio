// Package navigation implements the high-level navigation state machine for the
// golfbot.  It reads ball detections from the vision package and translates them
// into BeltDrive commands sent to the LEGO Mindstorms EV3.
package navigation

import (
	"context"
	"fmt"
	"math"
	"time"

	"bot/config"
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

// Navigator is the top-level controller that ties vision and motor control together.
type Navigator struct {
	drive    *mindstorm.BeltDrive
	collect  *mindstorm.Motor // the back/collector motor (may be nil)
	cam      *vision.Camera
	detector *vision.Detector

	// perspective is non-nil when perspective correction is configured.
	perspective *vision.PerspectiveTransform

	state        State
	lastSeen     time.Time
	collectStart time.Time

	// frameWidth / frameHeight are cached after the first frame (post-warp).
	frameWidth  int
	frameHeight int
}

// New creates a Navigator.  The collect motor may be nil if there is no separate
// collector mechanism.
//
// If vision.perspective.enabled is true in the global config, the perspective
// transform is initialised here.  Callers should call Close() on the returned
// Navigator when done to release resources.
func New(
	drive *mindstorm.BeltDrive,
	collect *mindstorm.Motor,
	cam *vision.Camera,
	detector *vision.Detector,
) *Navigator {
	n := &Navigator{
		drive:    drive,
		collect:  collect,
		cam:      cam,
		detector: detector,
		state:    StateSearching,
		lastSeen: time.Now(),
	}

	pcfg := config.Get().Vision.Perspective
	if pcfg.Enabled {
		corners := [4]vision.PerspectivePoint{
			{X: pcfg.TopLeft.X, Y: pcfg.TopLeft.Y},
			{X: pcfg.TopRight.X, Y: pcfg.TopRight.Y},
			{X: pcfg.BottomRight.X, Y: pcfg.BottomRight.Y},
			{X: pcfg.BottomLeft.X, Y: pcfg.BottomLeft.Y},
		}
		pt, err := vision.NewPerspectiveTransform(corners, pcfg.OutputWidth, pcfg.OutputHeight)
		if err != nil {
			log.WithError(err).Warn("navigation: perspective warp disabled — failed to init")
		} else {
			n.perspective = pt
			log.WithFields(log.Fields{
				"output_width":  pcfg.OutputWidth,
				"output_height": pcfg.OutputHeight,
			}).Info("navigation: perspective warp enabled")
		}
	}

	return n
}

// Close releases any resources held by the Navigator (e.g. the perspective
// homography matrix).
func (n *Navigator) Close() {
	if n.perspective != nil {
		n.perspective.Close()
		n.perspective = nil
	}
}

// Run starts the navigation loop and blocks until ctx is cancelled or a fatal
// error occurs.  It is safe to call Stop() from another goroutine while Run is
// executing.
func (n *Navigator) Run(ctx context.Context) error {
	frame := gocv.NewMat()
	defer frame.Close()

	// Pre-allocate a warp destination only when perspective correction is active.
	warped := gocv.NewMat()
	defer warped.Close()

	cfg := config.Get().Navigation
	ticker := time.NewTicker(time.Duration(cfg.TickIntervalMs) * time.Millisecond)
	defer ticker.Stop()

	log.WithField("state", n.state).Info("navigation: starting loop")

	for {
		select {
		case <-ctx.Done():
			log.Info("navigation: context cancelled, stopping")
			_ = n.drive.Stop()
			return ctx.Err()
		case <-ticker.C:
			if err := n.tick(&frame, &warped); err != nil {
				log.WithError(err).Error("navigation: tick error")
			}
		}
	}
}

// tick executes one control loop iteration.
//
// rawFrame is filled with the latest camera frame.  When perspective correction
// is configured, the frame is warped into warpBuf and warpBuf is passed to the
// detector; otherwise rawFrame is used directly.
func (n *Navigator) tick(rawFrame, warpBuf *gocv.Mat) error {
	if err := n.cam.Read(rawFrame); err != nil {
		return fmt.Errorf("navigation: read frame: %w", err)
	}

	// Determine which frame goes to the detector.
	detectFrame := rawFrame
	if n.perspective != nil {
		if err := n.perspective.Warp(*rawFrame, warpBuf); err != nil {
			// Log but fall back to the raw frame so the robot keeps running.
			log.WithError(err).Warn("navigation: perspective warp failed, using raw frame")
		} else {
			detectFrame = warpBuf
		}
	}

	// Cache the working frame dimensions on the first successful frame.
	if n.frameWidth == 0 {
		n.frameWidth = detectFrame.Cols()
		n.frameHeight = detectFrame.Rows()
	}

	balls, err := n.detector.Detect(*detectFrame)
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

	cfg := config.Get().Navigation
	return n.drive.Turn(cfg.TurnSpeed)
}

// handleApproaching steers the robot toward the closest detected ball.
func (n *Navigator) handleApproaching(balls []vision.Ball) error {
	cfg := config.Get().Navigation
	vcfg := config.Get().Vision

	if len(balls) == 0 {
		if time.Since(n.lastSeen) > time.Duration(cfg.SearchTimeoutMs)*time.Millisecond {
			log.Info("navigation: ball lost, returning to searching")
			n.transitionTo(StateSearching)
		}
		return nil
	}

	n.lastSeen = time.Now()
	target := balls[0] // largest = closest

	dist := target.EstimatedDistance(vcfg.DistanceK)

	log.WithFields(log.Fields{
		"dist":   fmt.Sprintf("%.2f", dist),
		"radius": fmt.Sprintf("%.1f", target.Radius),
		"norm_x": fmt.Sprintf("%.3f", target.NormX(n.frameWidth)),
	}).Debug("navigation: approaching ball")

	if dist <= cfg.CollectDistanceThreshold {
		log.Info("navigation: ball in range, switching to collecting")
		n.transitionTo(StateCollecting)
		return nil
	}

	// Proportional steering: offset in [-1,1] multiplied by gain gives turn correction.
	normX := target.NormX(n.frameWidth)
	turnCorrection := clamp(normX*cfg.SteeringGain, -1, 1)

	// Scale drive speed down when heavily off-axis to avoid overshooting.
	speedScale := 1.0 - math.Abs(normX)*0.4
	throttle := cfg.DriveSpeed * speedScale

	return n.drive.SetThrottle(throttle, turnCorrection)
}

// handleCollecting activates the collector motor and waits for the dwell time.
func (n *Navigator) handleCollecting() error {
	cfg := config.Get().Navigation
	dwellDur := time.Duration(cfg.CollectDwellMs) * time.Millisecond

	if n.collectStart.IsZero() {
		if err := n.drive.Stop(); err != nil {
			return err
		}
		if n.collect != nil {
			if err := n.collect.RunTimed(600, cfg.CollectDwellMs); err != nil {
				log.WithError(err).Warn("navigation: collector motor error")
			}
		}
		n.collectStart = time.Now()
		log.Info("navigation: collecting ball")
		return nil
	}

	if time.Since(n.collectStart) >= dwellDur {
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
