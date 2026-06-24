package main

import (
	"image"
	"image/color"
	"math"

	"gocv.io/x/gocv"
)

// RobotState holds everything the navigator needs to know about the robot.
type RobotState struct {
	Detected bool
	Center   image.Point
	Angle    float64        // Facing direction in degrees (0 to 360)
	Box      image.Rectangle // Bounding rectangle of the ArUco marker (used to exclude false-positive ball detections inside the purple square)
}

// RobotSpotter manages the ArUco tracking state.
type RobotSpotter struct {
	detector    gocv.ArucoDetector
	purpleColor color.RGBA
}

// NewRobotSpotter initializes the ArUco tracking configuration
func NewRobotSpotter() *RobotSpotter {
	dict := gocv.GetPredefinedDictionary(gocv.ArucoDict4x4_250)
	params := gocv.NewArucoDetectorParameters()

	return &RobotSpotter{
		detector:    gocv.NewArucoDetectorWithParams(dict, params),
		purpleColor: color.RGBA{255, 0, 255, 0},
	}
}

// TrackRobot scans the frame for ArUco markers and returns position/heading.
func (rs *RobotSpotter) TrackRobot(frame *gocv.Mat) RobotState {
	var robot RobotState

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

	// Center of marker.
	var sumX, sumY float32
	minX, minY := int(markerCorners[0].X), int(markerCorners[0].Y)
	maxX, maxY := minX, minY
	for _, pt := range markerCorners {
		sumX += float32(pt.X)
		sumY += float32(pt.Y)
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
	robot.Center = image.Pt(int(sumX/4), int(sumY/4))

	// Store bounding box with a small padding so the full purple square is covered.
	const boxPad = 10
	robot.Box = image.Rect(minX-boxPad, minY-boxPad, maxX+boxPad, maxY+boxPad)

	// Heading from corner 3 -> corner 0.
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

	// Purple marker outline.
	for j := 0; j < 4; j++ {
		p1 := markerCorners[j]
		p2 := markerCorners[(j+1)%4]

		pt1 := image.Pt(int(p1.X), int(p1.Y))
		pt2 := image.Pt(int(p2.X), int(p2.Y))

		gocv.Line(frame, pt1, pt2, rs.purpleColor, 3)
	}

	// Heading arrow.
	arrowLength := 40.0
	targetX := float64(robot.Center.X) + arrowLength*math.Cos(radians)
	targetY := float64(robot.Center.Y) + arrowLength*math.Sin(radians)
	arrowTarget := image.Pt(int(targetX), int(targetY))

	gocv.ArrowedLine(frame, robot.Center, arrowTarget, rs.purpleColor, 3)

	return robot
}
