package navigation

import (
	"bot/mindstorm"
	"math"
	"time"
)

// AngleToTarget returns the angle (degrees) the robot must turn to face a point.
// Positive = turn left, Negative = turn right
func AngleToTarget(robot RobotState, target Point) float64 {
	dx := target.X - robot.Position.X
	dy := target.Y - robot.Position.Y
	targetAngle := math.Atan2(dy, dx) * 180 / math.Pi
	delta := targetAngle - robot.Heading
	// Normalize to [-180, 180]
	for delta > 180 {
		delta -= 360
	}
	for delta < -180 {
		delta += 360
	}
	return delta
}

func NavigateTo(drive *mindstorm.BeltDrive, state CourseState, target Point) error {
	angle := AngleToTarget(state.Robot, target)

	// Turn in place: one motor forward, one backward
	if math.Abs(angle) > 5 { // 5° dead zone to avoid jitter
		if angle > 0 {
			drive.TurnLeft(0.3)
		} else {
			drive.TurnRight(0.3)
		}
		// Duration proportional to angle — needs calibration on real hardware
		time.Sleep(time.Duration(math.Abs(angle)*10) * time.Millisecond)
		drive.Stop()
	}

	// Drive forward toward target
	dist := Distance(state.Robot.Position, target)
	drive.Drive(0.4)
	time.Sleep(time.Duration(dist*20) * time.Millisecond) // calibrate multiplier
	drive.Stop()
	return nil
}
