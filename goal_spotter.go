package main

import (
	"image"
	"image/color"
	"math"

	"gocv.io/x/gocv"
)

// GoalData holds the location and orientation of the goal ArUco marker (ID 1).
type GoalData struct {
	Detected bool
	Center   image.Point
	Box      image.Rectangle
	// FaceAngle is the outward-facing normal of the marker in image-space degrees
	// (clockwise from +X, 0-360). The robot should reverse along FaceAngle+180 to
	// approach perpendicularly from the front of the marker.
	// Derived from the ArUco corner[0]->corner[1] edge normal.
	FaceAngle float64
}

// GoalSpotter manages tracking for the disposal area
type GoalSpotter struct {
	detector  gocv.ArucoDetector
	cyanColor color.RGBA
}

// NewGoalSpotter initializes tracking for ArUco Marker ID 1.
// ID 0 is reserved for the robot marker (RobotSpotter).
func NewGoalSpotter() *GoalSpotter {
	dict := gocv.GetPredefinedDictionary(gocv.ArucoDict4x4_250)
	params := gocv.NewArucoDetectorParameters()

	// Adjust parameters to handle board glare matching our working setup
	params.SetMinMarkerPerimeterRate(0.05)
	params.SetAdaptiveThreshWinSizeMin(3)
	params.SetAdaptiveThreshWinSizeMax(23)
	params.SetAdaptiveThreshWinSizeStep(10)
	params.SetPolygonalApproxAccuracyRate(0.03)

	return &GoalSpotter{
		detector:  gocv.NewArucoDetectorWithParams(dict, params),
		cyanColor: color.RGBA{0, 255, 255, 0},
	}
}

// TrackGoal scans the frame for ArUco Marker ID 1 (the goal/disposal zone).
// It populates FaceAngle with the outward normal of the marker face so the
// delivery FSM can align the robot perpendicularly before reversing in.
func (gs *GoalSpotter) TrackGoal(frame *gocv.Mat) GoalData {
	var goal GoalData

	corners, ids, _ := gs.detector.DetectMarkers(*frame)

	// Find marker ID 1.
	targetIndex := -1
	for idx, id := range ids {
		if id == 1 {
			targetIndex = idx
			break
		}
	}

	if targetIndex == -1 {
		return goal
	}

	goal.Detected = true
	goalCorners := corners[targetIndex]

	if len(goalCorners) < 4 {
		return goal
	}

	minX, minY := int(goalCorners[0].X), int(goalCorners[0].Y)
	maxX, maxY := minX, minY

	var sumX, sumY float32
	for _, pt := range goalCorners {
		sumX += pt.X
		sumY += pt.Y

		if int(pt.X) < minX {
			minX = int(pt.X)
		}
		if int(pt.X) > maxX {
			maxX = int(pt.X)
		}
		if int(pt.Y) < minY {
			minY = int(pt.Y)
		}
		if int(pt.Y) > maxY {
			maxY = int(pt.Y)
		}
	}

	goal.Center = image.Pt(int(sumX/4), int(sumY/4))
	goal.Box = image.Rect(minX, minY, maxX, maxY)

	// ── Compute the outward face normal ──────────────────────────────────────
	// ArUco corners are ordered: top-left(0), top-right(1), bottom-right(2), bottom-left(3)
	// The "top" edge runs from corner[0] to corner[1].
	// Its outward normal (pointing away from the board, i.e. toward the camera
	// / the approaching robot) is 90° counter-clockwise from the edge direction.
	//
	//   edge vector:   (dx, dy) = corner[1] - corner[0]
	//   outward normal (CCW 90°): (-dy, dx)
	//
	// We convert this to the same clockwise-from-+X convention used everywhere else.
	edgeDX := float64(goalCorners[1].X - goalCorners[0].X)
	edgeDY := float64(goalCorners[1].Y - goalCorners[0].Y)
	normalDX := -edgeDY
	normalDY := edgeDX
	faceAngleDeg := math.Atan2(normalDY, normalDX) * 180.0 / math.Pi
	if faceAngleDeg < 0 {
		faceAngleDeg += 360
	}
	goal.FaceAngle = faceAngleDeg

	// ── Draw overlay ────────────────────────────────────────────────────────
	gocv.Rectangle(frame, goal.Box, gs.cyanColor, 3)
	gocv.Circle(frame, goal.Center, 5, gs.cyanColor, -1)
	gocv.PutText(frame, "GOAL [ID 1]", image.Pt(minX, minY-10), gocv.FontHersheySimplex, 0.5, gs.cyanColor, 2)

	// Draw the face-normal arrow (shows which direction the robot should approach from).
	arrowLen := 40.0
	arrowTip := image.Pt(
		goal.Center.X+int(math.Cos(math.Atan2(normalDY, normalDX))*arrowLen),
		goal.Center.Y+int(math.Sin(math.Atan2(normalDY, normalDX))*arrowLen),
	)
	gocv.ArrowedLine(frame, goal.Center, arrowTip, gs.cyanColor, 2)

	return goal
}
