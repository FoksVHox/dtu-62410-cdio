package main

import (
	"image"
	"image/color"
	"math"

	"gocv.io/x/gocv"
)

// RobotState holds the detected position and heading of the robot in pixel space.
type RobotState struct {
	Detected bool
	Center   image.Point
	// Angle is the heading in degrees, measured clockwise from the positive X axis (right).
	// 0° = facing right, 90° = facing down, 180° = facing left, 270° = facing up.
	Angle float64
}

// RobotSpotter detects the robot from a top-down camera image.
// The robot must have a large RED marker for its body center and a smaller BLUE marker
// offset forward so the heading can be computed.
type RobotSpotter struct {
	// Nothing stateful yet, but kept as struct so future filtering (e.g. Kalman) can be added.
}

// NewRobotSpotter creates a new RobotSpotter.
func NewRobotSpotter() *RobotSpotter {
	return &RobotSpotter{}
}

// TrackRobot analyses the current frame and returns the robot state.
// It expects a BGR image. The function draws debug overlays onto the frame.
func (rs *RobotSpotter) TrackRobot(frame *gocv.Mat) RobotState {
	hsv := gocv.NewMat()
	defer hsv.Close()
	gocv.CvtColor(*frame, &hsv, gocv.ColorBGRToHSV)

	// ---- detect large RED body marker ----
	mask1 := gocv.NewMat()
	defer mask1.Close()
	mask2 := gocv.NewMat()
	defer mask2.Close()
	redMask := gocv.NewMat()
	defer redMask.Close()

	gocv.InRangeWithScalar(hsv, gocv.NewScalar(0, 120, 120, 0), gocv.NewScalar(10, 255, 255, 0), &mask1)
	gocv.InRangeWithScalar(hsv, gocv.NewScalar(170, 120, 120, 0), gocv.NewScalar(180, 255, 255, 0), &mask2)
	gocv.BitwiseOr(mask1, mask2, &redMask)

	body, bodyOK := largestContourCenter(redMask, 300)

	// ---- detect small BLUE heading marker ----
	blueMask := gocv.NewMat()
	defer blueMask.Close()
	gocv.InRangeWithScalar(hsv, gocv.NewScalar(100, 150, 100, 0), gocv.NewScalar(130, 255, 255, 0), &blueMask)

	heading, headingOK := largestContourCenter(blueMask, 50)

	if !bodyOK {
		return RobotState{Detected: false}
	}

	angle := 0.0
	if headingOK {
		dx := float64(heading.X - body.X)
		dy := float64(heading.Y - body.Y)
		// atan2 gives angle from positive X axis; Y increases downward in image coords.
		angle = math.Atan2(dy, dx) * 180.0 / math.Pi
		if angle < 0 {
			angle += 360
		}
	}

	// Draw overlays.
	redDot := color.RGBA{0, 0, 255, 0} // BGR red
	blueDot := color.RGBA{255, 0, 0, 0} // BGR blue
	gocv.Circle(frame, body, 8, redDot, -1)
	if headingOK {
		gocv.Circle(frame, heading, 5, blueDot, -1)
		gocv.Line(frame, body, heading, blueDot, 2)
	}

	return RobotState{
		Detected: true,
		Center:   body,
		Angle:    angle,
	}
}

// largestContourCenter returns the centroid of the largest contour in mask that
// exceeds minArea pixels. Returns false if none found.
func largestContourCenter(mask gocv.Mat, minArea float64) (image.Point, bool) {
	contours := gocv.FindContours(mask, gocv.RetrievalExternal, gocv.ChainApproxSimple)
	defer contours.Close()

	bestArea := 0.0
	bestIdx := -1
	for i := 0; i < contours.Size(); i++ {
		a := gocv.ContourArea(contours.At(i))
		if a > bestArea {
			bestArea = a
			bestIdx = i
		}
	}
	if bestIdx < 0 || bestArea < minArea {
		return image.Point{}, false
	}
	r := gocv.BoundingRect(contours.At(bestIdx))
	return image.Pt(r.Min.X+r.Dx()/2, r.Min.Y+r.Dy()/2), true
}
