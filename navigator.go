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
//  1. COARSE ALIGN  — robot is stationary; turn in place until heading error < DeadBandDeg.
//     The turn DIRECTION is locked on entry and never flips, preventing
//     ±180° wrap-around oscillation.
//  2. DRIVE         — move forward at full DriveSpeed; apply a gentle steering correction
//     proportional to the heading error.  Stay in DRIVE as long as the
//     error stays below ReAlignDeg.  If the robot drifts beyond ReAlignDeg
//     it drops back to COARSE ALIGN and stops.
//  3. FINE ALIGN    — when dist < FineAlignDist the steering correction is scaled up so the
//     robot arrives at the ball from directly in front rather than at an angle.
type Navigator struct {
	// CoarseAlignDeg: heading error (degrees) at which the proportional turn reaches TurnSpeed.
	CoarseAlignDeg float64
	// DeadBandDeg: heading error below which coarse-align is considered done and DRIVE begins.
	DeadBandDeg float64
	// ReAlignDeg: if the heading error exceeds this while driving we stop and realign.
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
	SteerGain float64

	// internal state
	driving      bool
	turnSign     float64 // locked turn direction during coarse-align: +1 or -1
	lastTargetX  int
	lastTargetY  int
	targetInited bool
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
		CoarseAlignDeg:    15.0, // proportional ramp reaches TurnSpeed at 15°
		DeadBandDeg:       7.0,  // enter DRIVE once within 3° — tight but stable
		ReAlignDeg:        20.0, // stop driving and realign if drift exceeds 20°
		FineAlignDist:     80.0,
		ArrivedRadius:     30.0,
		GoalArrivedRadius: 60.0,
		DriveSpeed:        0.5,
		TurnSpeed:         0.4,
		SteerGain:         0.025,
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

	// --- 0. Detect target change and reset state ---
	if !n.targetInited || target.X != n.lastTargetX || target.Y != n.lastTargetY {
		fmt.Printf("[NAV] TARGET CHANGED (%d,%d) -> (%d,%d) — reset to COARSE_ALIGN\n",
			n.lastTargetX, n.lastTargetY, target.X, target.Y)
		n.driving = false
		n.turnSign = 0
		n.lastTargetX = target.X
		n.lastTargetY = target.Y
		n.targetInited = true
	}

	// --- 1. Distance to target ---
	dx := float64(target.X - robot.Center.X)
	dy := float64(target.Y - robot.Center.Y)
	dist := math.Sqrt(dx*dx + dy*dy)

	if dist <= arrivedRadius {
		fmt.Printf("[NAV] ARRIVED — dist=%.1fpx <= radius=%.1fpx\n", dist, arrivedRadius)
		n.driving = false
		n.turnSign = 0
		return DriveCommand{Arrived: true}, nil
	}

	// --- 2. Bearing to target (degrees, clockwise from +X, 0..360) ---
	bearingDeg := math.Atan2(dy, dx) * 180.0 / math.Pi
	if bearingDeg < 0 {
		bearingDeg += 360
	}

	// --- 3. Heading error in (-180, 180] ---
	// normaliseAngle always gives the SHORTEST path, so its sign on the very
	// first call is the correct turn direction. We lock that sign for the
	// entire coarse-align phase so the ±180° boundary never flips us.
	rawErr := normaliseAngle(bearingDeg - robot.Angle)
	absErr := math.Abs(rawErr)

	// --- 4. Stage transitions ---
	prevDriving := n.driving
	if n.driving {
		if absErr >= n.ReAlignDeg {
			n.driving = false
			n.turnSign = 0
		}
	} else {
		// Lock turn direction on first frame of coarse-align.
		if n.turnSign == 0 && rawErr != 0 {
			n.turnSign = math.Copysign(1, rawErr)
			fmt.Printf("[NAV] COARSE_ALIGN locked turnSign=%.0f (rawErr=%.2f°)\n", n.turnSign, rawErr)
		}
		if absErr < n.DeadBandDeg {
			n.driving = true
			n.turnSign = 0
		}
	}
	if prevDriving != n.driving {
		if n.driving {
			fmt.Printf("[NAV] STATE -> DRIVE (absErr=%.2f° < deadBand=%.2f°)\n", absErr, n.DeadBandDeg)
		} else {
			fmt.Printf("[NAV] STATE -> COARSE_ALIGN (absErr=%.2f° >= reAlign=%.2f°)\n", absErr, n.ReAlignDeg)
		}
	}

	// --- 5a. COARSE ALIGN — turn in place, zero forward throttle ---
	// Magnitude tapers proportionally to zero as absErr → 0.
	// Direction is n.turnSign (locked) — never reads rawErr sign again.
	if !n.driving {
		norm := math.Min(absErr/n.CoarseAlignDeg, 1.0)
		mag := n.TurnSpeed * norm
		turnOut := n.turnSign * mag
		fmt.Printf("[NAV] COARSE_ALIGN | robotAngle=%.1f° bearing=%.1f° rawErr=%.2f° absErr=%.2f° turnSign=%.0f | turn=%.3f\n",
			robot.Angle, bearingDeg, rawErr, absErr, n.turnSign, turnOut)
		return DriveCommand{Throttle: 0, Turn: turnOut}, nil
	}

	// --- 5b. DRIVE — forward with proportional steering correction ---
	gain := n.SteerGain
	if dist < n.FineAlignDist {
		fineRatio := 1.0 - (dist-arrivedRadius)/(n.FineAlignDist-arrivedRadius)
		fineRatio = math.Max(0, math.Min(1, fineRatio))
		gain = n.SteerGain * (1.0 + 2.0*fineRatio)
	}
	correction := rawErr * gain
	correction = math.Max(-n.TurnSpeed, math.Min(n.TurnSpeed, correction))
	throttle := n.DriveSpeed * (1.0 - 0.35*math.Abs(correction)/n.TurnSpeed)

	fmt.Printf("[NAV] DRIVE | robotAngle=%.1f° bearing=%.1f° rawErr=%.2f° dist=%.1fpx | thr=%.3f turn=%.3f\n",
		robot.Angle, bearingDeg, rawErr, dist, throttle, correction)
	return DriveCommand{Throttle: throttle, Turn: correction}, nil
}

// PickNextBall selects the best ball to collect next.
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
	return fmt.Sprintf("NAV: dist=%.0fpx heading=%.0f° thr=%.2f turn=%.2f",
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
