package main

import (
	"fmt"
	"image"
	"math"
)

// Navigator holds navigation configuration and exposes methods to compute
// the drive commands needed to reach a target ball or goal from the current
// robot state.
//
// Coordinate system: top-down image, X increases rightward, Y increases downward.
// Angles are in degrees, clockwise from the positive-X axis (matching RobotSpotter).
//
// Movement strategy — three stages:
//
//	1. COARSE ALIGN  — robot is stationary; turn in place until heading error < CoarseAlignDeg.
//	2. DRIVE         — move forward at full DriveSpeed; apply a gentle steering correction
//	                   proportional to the heading error.  Stay in DRIVE as long as the error
//	                   stays below ReAlignDeg.  If the robot drifts beyond ReAlignDeg it drops
//	                   back to COARSE ALIGN and stops.
//	3. FINE ALIGN    — when dist < FineAlignDist the steering correction is scaled up so the
//	                   robot arrives at the ball from directly in front rather than at an angle.
type Navigator struct {
	// CoarseAlignDeg: heading error (degrees) below which we leave turn-in-place and
	// start driving. Set to 5 so the robot commits to a direction before moving.
	CoarseAlignDeg float64
	// ReAlignDeg: if the heading error exceeds this while driving we stop and realign.
	// Must be > CoarseAlignDeg to create hysteresis and avoid rapid mode switching.
	ReAlignDeg float64
	// FineAlignDist: pixel distance at which fine-alignment steering kicks in.
	FineAlignDist float64
	// ArrivedRadius is the pixel distance at which we consider the ball collected.
	ArrivedRadius float64
	// GoalArrivedRadius is the pixel distance at which we consider the robot
	// close enough to the goal to release the ball(s).
	GoalArrivedRadius float64
	// DriveSpeed is the forward throttle [0, 1] while moving.
	DriveSpeed float64
	// TurnSpeed is the maximum turn magnitude [0, 1] used during coarse alignment.
	TurnSpeed float64
	// SteerGain scales the proportional steering correction while driving.
	// Lower values = gentler curves; higher values = tighter corrections.
	SteerGain float64

	// internal state
	driving bool
}

// DriveCommand is the output of the navigator.
type DriveCommand struct {
	// Throttle in [-1, 1]. Positive = forward.
	Throttle float64
	// Turn in [-1, 1]. Positive = clockwise (right). Negative = counter-clockwise.
	Turn float64
	// Arrived is true when the robot is close enough to collect / deposit.
	Arrived bool
}

// NewNavigator creates a Navigator with tuned defaults.
func NewNavigator() *Navigator {
	return &Navigator{
		CoarseAlignDeg:    5.0,  // stop turning-in-place once within 5 degrees
		ReAlignDeg:        20.0, // re-enter coarse-align only if drift exceeds 20 degrees
		FineAlignDist:     80.0, // pixels — engage stronger correction in the last 80 px
		ArrivedRadius:     30.0, // pixels — ball collection threshold
		GoalArrivedRadius: 60.0, // pixels — goal deposit threshold
		DriveSpeed:        0.5,
		TurnSpeed:         0.4,
		SteerGain:         0.025, // turn per degree of heading error while driving
	}
}

// NextCommand computes the drive command to reach a ball.
func (n *Navigator) NextCommand(robot RobotState, target Ball) (DriveCommand, error) {
	return n.navigateTo(robot, target.Center, n.ArrivedRadius)
}

// NextCommandToPoint computes the drive command to reach an arbitrary pixel-space point.
func (n *Navigator) NextCommandToPoint(robot RobotState, target image.Point) (DriveCommand, error) {
	return n.navigateTo(robot, target, n.GoalArrivedRadius)
}

// navigateTo is the shared steering core.
func (n *Navigator) navigateTo(robot RobotState, target image.Point, arrivedRadius float64) (DriveCommand, error) {
	if !robot.Detected {
		return DriveCommand{}, fmt.Errorf("navigator: robot not detected")
	}

	// --- 1. Distance to target ---
	dx := float64(target.X - robot.Center.X)
	dy := float64(target.Y - robot.Center.Y)
	dist := math.Sqrt(dx*dx + dy*dy)

	if dist <= arrivedRadius {
		n.driving = false
		return DriveCommand{Arrived: true}, nil
	}

	// --- 2. Bearing to target (degrees, clockwise from +X, 0..360) ---
	bearingDeg := math.Atan2(dy, dx) * 180.0 / math.Pi
	if bearingDeg < 0 {
		bearingDeg += 360
	}

	// --- 3. Heading error in (-180, 180] ---
	// Positive = target is to our right (need clockwise / right turn).
	headingErr := normaliseAngle(bearingDeg - robot.Angle)
	absErr := math.Abs(headingErr)

	// --- 4. Stage transitions ---
	//
	// COARSE ALIGN  (driving == false):
	//   Transition to DRIVE when absErr < CoarseAlignDeg.
	//
	// DRIVE         (driving == true):
	//   Stay in DRIVE as long as absErr < ReAlignDeg.
	//   Drop back to COARSE ALIGN if absErr >= ReAlignDeg.
	//
	// This wide hysteresis band (5 deg → 20 deg) means the robot will not
	// stop to realign unless it has drifted seriously, so it keeps moving
	// forward the vast majority of the time.
	if n.driving {
		if absErr >= n.ReAlignDeg {
			n.driving = false
		}
	} else {
		if absErr < n.CoarseAlignDeg {
			n.driving = true
		}
	}

	// --- 5a. COARSE ALIGN — turn in place, no forward throttle ---
	if !n.driving {
		// Proportional turn: fast when badly mis-aligned, slows near target heading.
		// Clamped to TurnSpeed; small minimum prevents motor stall.
		norm := math.Min(absErr/90.0, 1.0) // 0..1 over the first 90 degrees
		mag := n.TurnSpeed*norm + 0.1       // linear ramp + small dead-zone lift
		if mag > n.TurnSpeed {
			mag = n.TurnSpeed
		}
		turnDir := math.Copysign(mag, headingErr)
		return DriveCommand{Throttle: 0, Turn: turnDir}, nil
	}

	// --- 5b. DRIVE — move forward with gentle proportional steering correction ---
	//
	// Normal gain while far away (> FineAlignDist).
	// Boosted gain when close (< FineAlignDist) so the final approach is straight.
	gain := n.SteerGain
	if dist < n.FineAlignDist {
		// Linearly increase gain as we close in, up to 3x at arrivedRadius.
		fineRatio := 1.0 - (dist-arrivedRadius)/(n.FineAlignDist-arrivedRadius)
		fineRatio = math.Max(0, math.Min(1, fineRatio))
		gain = n.SteerGain * (1.0 + 2.0*fineRatio)
	}

	correction := headingErr * gain
	correction = math.Max(-n.TurnSpeed, math.Min(n.TurnSpeed, correction))

	// Reduce speed slightly when a large correction is needed so the robot arcs
	// smoothly rather than ploughing past the target.
	throttle := n.DriveSpeed * (1.0 - 0.35*math.Abs(correction)/n.TurnSpeed)

	return DriveCommand{Throttle: throttle, Turn: correction}, nil
}

// PickNextBall selects the best ball to collect next.
//   - If an orange (VIP) ball exists and has not been delivered, it is returned first.
//   - Otherwise the nearest reachable (non-red-zone) ball is returned.
//
// Returns nil if no reachable ball is available.
func PickNextBall(robot RobotState, balls []Ball, orangeDelivered bool) *Ball {
	if !robot.Detected || len(balls) == 0 {
		return nil
	}

	if !orangeDelivered {
		for i := range balls {
			if balls[i].IsOrange && !balls[i].InRedZone {
				return &balls[i]
			}
		}
	}

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

// DebugNavigation returns a human-readable nav summary for the overlay.
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

// ArrowPoints returns start/end image.Points for a directional arrow from 'from' toward 'to'.
func ArrowPoints(from, to image.Point, length float64) (image.Point, image.Point) {
	dx := float64(to.X - from.X)
	dy := float64(to.Y - from.Y)
	d := math.Sqrt(dx*dx + dy*dy)
	if d == 0 {
		return from, to
	}
	if length > d {
		length = d
	}
	return from, image.Pt(
		from.X+int(dx/d*length),
		from.Y+int(dy/d*length),
	)
}
