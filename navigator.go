package main

import (
	"fmt"
	"image"
	"math"
)

// Navigator holds navigation configuration and exposes methods to compute
// the drive commands needed to reach a target ball from the current robot state.
//
// Coordinate system: top-down image, X increases rightward, Y increases downward.
// Angles are in degrees, clockwise from the positive-X axis (matching RobotSpotter).
type Navigator struct {
	// TurnThreshold is the angle error (degrees) below which we stop turning and
	// drive straight toward the target.
	TurnThreshold float64
	// ArrivedRadius is the pixel distance at which we consider the ball collected.
	ArrivedRadius float64
	// GoalArrivedRadius is the pixel distance at which we consider the robot
	// close enough to the goal to release the ball(s). Larger than ArrivedRadius
	// so the robot doesn't ram the goal marker.
	GoalArrivedRadius float64
	// DriveSpeed is the base throttle in the range [0, 1] used while moving forward.
	DriveSpeed float64
	// TurnSpeed is the turn magnitude in the range [0, 1] used while rotating in place.
	TurnSpeed float64
}

// DriveCommand is the output of the navigator: a throttle + turn pair for BeltDrive.
type DriveCommand struct {
	// Throttle in [-1, 1]. Positive = forward.
	Throttle float64
	// Turn in [-1, 1]. Positive = clockwise (right turn). Negative = counter-clockwise.
	Turn float64
	// Arrived is true when the robot is close enough to the ball to pick it up.
	Arrived bool
}

// NewNavigator creates a Navigator with sensible defaults.
func NewNavigator() *Navigator {
	return &Navigator{
		TurnThreshold:     10.0, // degrees
		ArrivedRadius:     30.0, // pixels — ball collection
		GoalArrivedRadius: 60.0, // pixels — goal deposit (stop before hitting the marker)
		DriveSpeed:        0.5,
		TurnSpeed:         0.4,
	}
}

// NextCommand computes the drive command for one control loop tick.
//
//   - robot: current robot state from RobotSpotter.
//   - target: the ball we want to reach.
//
// Returns (DriveCommand, nil) on success, or (zero, error) if the robot is not detected.
func (n *Navigator) NextCommand(robot RobotState, target Ball) (DriveCommand, error) {
	return n.navigateTo(robot, target.Center, n.ArrivedRadius)
}

// NextCommandToPoint computes the drive command to reach an arbitrary pixel-space
// point (e.g. the goal marker centre). Uses GoalArrivedRadius so the robot stops
// before physically colliding with the goal marker.
//
// Returns (DriveCommand, nil) on success, or (zero, error) if the robot is not detected.
func (n *Navigator) NextCommandToPoint(robot RobotState, target image.Point) (DriveCommand, error) {
	return n.navigateTo(robot, target, n.GoalArrivedRadius)
}

// navigateTo is the shared steering core used by both NextCommand and NextCommandToPoint.
func (n *Navigator) navigateTo(robot RobotState, target image.Point, arrivedRadius float64) (DriveCommand, error) {
	if !robot.Detected {
		return DriveCommand{}, fmt.Errorf("navigator: robot not detected")
	}

	// --- 1. Distance to target ---
	dx := float64(target.X - robot.Center.X)
	dy := float64(target.Y - robot.Center.Y)
	dist := math.Sqrt(dx*dx + dy*dy)

	if dist <= arrivedRadius {
		return DriveCommand{Arrived: true}, nil
	}

	// --- 2. Bearing to target ---
	// math.Atan2 returns angle in [-π, π] from positive-X axis, clockwise in image coords.
	bearingRad := math.Atan2(dy, dx)
	bearingDeg := bearingRad * 180.0 / math.Pi
	if bearingDeg < 0 {
		bearingDeg += 360
	}

	// --- 3. Heading error ---
	// Positive error means the target is to our right (we need to turn clockwise).
	headingErr := bearingDeg - robot.Angle
	// Normalise to (-180, 180]
	headingErr = normaliseAngle(headingErr)

	// --- 4. Build command ---
	if math.Abs(headingErr) > n.TurnThreshold {
		// Turn in place toward the target. Scale magnitude with the error so we
		// spin quickly when badly misaligned and gently when nearly aligned.
		mag := n.TurnSpeed
		if a := math.Abs(headingErr); a < 45 {
			// Ramp down below 45° but keep a minimum so we don't stall.
			mag = math.Max(n.TurnSpeed*(a/45.0), 0.2)
		}
		turnDir := math.Copysign(mag, headingErr)
		return DriveCommand{Throttle: 0, Turn: turnDir}, nil
	}

	// Aligned enough — drive forward with a proportional steering correction.
	correction := (headingErr / n.TurnThreshold) * n.TurnSpeed
	correction = math.Max(-n.TurnSpeed, math.Min(n.TurnSpeed, correction))
	return DriveCommand{Throttle: n.DriveSpeed, Turn: correction}, nil
}

// PickNextBall selects the best ball to collect next:
//   - If an orange (VIP) ball exists and has not yet been delivered, it is
//     always returned first regardless of distance (bonus 200 pts).
//   - Otherwise the nearest reachable (non-red-zone) ball is returned.
//
// Returns nil if no reachable ball is available.
func PickNextBall(robot RobotState, balls []Ball, orangeDelivered bool) *Ball {
	if !robot.Detected || len(balls) == 0 {
		return nil
	}

	// If the orange ball hasn't been delivered yet, always go for it first.
	if !orangeDelivered {
		for i := range balls {
			if balls[i].IsOrange && !balls[i].InRedZone {
				return &balls[i]
			}
		}
	}

	// Fallback: nearest non-red-zone ball.
	var best *Ball
	bestDist := math.MaxFloat64
	for i := range balls {
		b := &balls[i]
		if b.InRedZone {
			continue
		}
		dx := float64(b.Center.X - robot.Center.X)
		dy := float64(b.Center.Y - robot.Center.Y)
		d := math.Sqrt(dx*dx + dy*dy)
		if d < bestDist {
			bestDist = d
			best = b
		}
	}
	return best
}

// normaliseAngle wraps an angle (degrees) into the range (-180, 180].
func normaliseAngle(a float64) float64 {
	for a > 180 {
		a -= 360
	}
	for a <= -180 {
		a += 360
	}
	return a
}

// DebugNavigation returns a human-readable summary of the navigation state for overlay text.
func DebugNavigation(robot RobotState, target *Ball, cmd DriveCommand) string {
	if !robot.Detected {
		return "NAV: robot not found"
	}
	if target == nil {
		return "NAV: no target"
	}
	dx := float64(target.Center.X - robot.Center.X)
	dy := float64(target.Center.Y - robot.Center.Y)
	dist := math.Sqrt(dx*dx + dy*dy)
	if cmd.Arrived {
		return fmt.Sprintf("NAV: ARRIVED at (%d,%d)", target.Center.X, target.Center.Y)
	}
	return fmt.Sprintf("NAV: dist=%.0fpx heading=%.0f\u00b0 thr=%.2f turn=%.2f",
		dist, robot.Angle, cmd.Throttle, cmd.Turn)
}

// ArrowPoints draws a directional arrow from the robot toward its target
// on an image (useful for debug overlay). Uses only stdlib image.Point.
func ArrowPoints(from, to image.Point, length float64) (image.Point, image.Point) {
	dx := float64(to.X - from.X)
	dy := float64(to.Y - from.Y)
	d := math.Sqrt(dx*dx + dy*dy)
	if d == 0 {
		return from, to
	}
	// Clamp arrow length.
	if length > d {
		length = d
	}
	return from, image.Pt(
		from.X+int(dx/d*length),
		from.Y+int(dy/d*length),
	)
}
