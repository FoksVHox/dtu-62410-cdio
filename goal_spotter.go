package main

import (
	"image"
	"image/color"

	"gocv.io/x/gocv"
)

// GoalData holds the location of our drop-off zone
type GoalData struct {
	Detected bool
	Center   image.Point
	Box      image.Rectangle
}

// GoalSpotter manages tracking for the disposal area
type GoalSpotter struct {
	detector  gocv.ArucoDetector
	cyanColor color.RGBA
}

// NewGoalSpotter initializes tracking for Marker ID 2
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
		cyanColor: color.RGBA{0, 255, 255, 0}, // Cyan blue for goal assets
	}
}

// TrackGoal scans the frame specifically looking for Marker ID 2
func (gs *GoalSpotter) TrackGoal(frame *gocv.Mat) GoalData {
	var goal GoalData

	corners, ids, _ := gs.detector.DetectMarkers(*frame)

	// Find if Marker ID 2 is anywhere in the detected list
	targetIndex := -1
	for idx, id := range ids {
		if id == 1 {
			targetIndex = idx
			break
		}
	}

	// If Marker ID 2 wasn't found, exit early
	if targetIndex == -1 {
		return goal
	}

	goal.Detected = true
	goalCorners := corners[targetIndex]

	if len(goalCorners) < 4 {
		return goal
	}

	// 1. Calculate bounding rectangle around the goal marker
	// We use the first corner and build a rectangle from the points
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

	// 2. Visuals: Draw a cyan target box around the goal zone slot
	gocv.Rectangle(frame, goal.Box, gs.cyanColor, 3)
	gocv.Circle(frame, goal.Center, 5, gs.cyanColor, -1)
	gocv.PutText(frame, "DISPOSAL GOAL", image.Pt(minX, minY-10), gocv.FontHersheySimplex, 0.5, gs.cyanColor, 2)

	return goal
}
