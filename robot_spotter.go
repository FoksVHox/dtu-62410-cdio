package main

import (
	"image"
	"image/color"
	"math"

	"gocv.io/x/gocv"
)

// RobotData holds everything we need to know about our LEGO picker
type RobotData struct {
	Detected bool
	Center   image.Point
	Angle    float64 // Facing direction in degrees (0 to 360)
}

// RobotSpotter manages the ArUco tracking states
type RobotSpotter struct {
	detector    gocv.ArucoDetector
	purpleColor color.RGBA
}

// NewRobotSpotter initializes our robot tracking configuration
func NewRobotSpotter() *RobotSpotter {
	// TEMPORARY TEST: Look for ANY 4x4 marker from the massive 250-count index
	dict := gocv.GetPredefinedDictionary(gocv.ArucoDict4x4_250)
	params := gocv.NewArucoDetectorParameters()

	return &RobotSpotter{
		detector:    gocv.NewArucoDetectorWithParams(dict, params),
		purpleColor: color.RGBA{255, 0, 255, 0}, // Regal purple
	}
}

// TrackRobot scans the frame for ArUco markers and returns position/heading
func (rs *RobotSpotter) TrackRobot(frame *gocv.Mat) RobotData {
	var robot RobotData

	// Detect markers, but this time we capture the 'rejected' list too!
	corners, ids, rejected := rs.detector.DetectMarkers(*frame)

	// ==========================================
	// DEBUG TOOL: Draw RED boxes around rejected shapes
	// ==========================================
	for _, rej := range rejected {
		if len(rej) == 4 {
			for j := 0; j < 4; j++ {
				pt1 := image.Pt(int(rej[j].X), int(rej[j].Y))
				pt2 := image.Pt(int(rej[(j+1)%4].X), int(rej[(j+1)%4].Y))
				gocv.Line(frame, pt1, pt2, color.RGBA{255, 0, 0, 0}, 2) // Draw Red lines
			}
		}
	}
	// ==========================================

	// NEW: Find if Marker ID 1 is anywhere in the detected list
	targetIndex := -1
	for idx, id := range ids {
		if id == 0 {
			targetIndex = idx
			break
		}
	}

	// If Marker ID 1 wasn't found, exit early!
	if targetIndex == -1 {
		return robot
	}

	robot.Detected = true
	// Grab the corners of ONLY Marker ID 1
	markerCorners := corners[targetIndex]

	if len(markerCorners) < 4 {
		return robot
	}

	// 1. Calculate Center of the marker
	var sumX, sumY float32
	for _, pt := range markerCorners {
		sumX += float32(pt.X)
		sumY += float32(pt.Y)
	}
	robot.Center = image.Pt(int(sumX/4), int(sumY/4))

	// 2. Calculate Heading Angle
	pFront := markerCorners[0]
	pBack := markerCorners[3]

	deltaX := float64(pFront.X - pBack.X)
	deltaY := float64(pFront.Y - pBack.Y)

	radians := math.Atan2(deltaY, deltaX)
	degrees := radians * (180.0 / math.Pi)

	if degrees < 0 {
		degrees += 360
	}
	robot.Angle = degrees

	// 3. Visuals: Draw the purple bounding frame
	for j := 0; j < 4; j++ {
		p1 := markerCorners[j]
		p2 := markerCorners[(j+1)%4]

		pt1 := image.Pt(int(p1.X), int(p1.Y))
		pt2 := image.Pt(int(p2.X), int(p2.Y))

		gocv.Line(frame, pt1, pt2, rs.purpleColor, 3)
	}

	arrowLength := 40.0
	targetX := float64(robot.Center.X) + arrowLength*math.Cos(radians)
	targetY := float64(robot.Center.Y) + arrowLength*math.Sin(radians)
	arrowTarget := image.Pt(int(targetX), int(targetY))

	gocv.ArrowedLine(frame, robot.Center, arrowTarget, rs.purpleColor, 3)

	return robot
}
