package main

import (
	"fmt"
	"image"
	"image/color"
	"os"

	"gocv.io/x/gocv"
)

func main() {
	webcam, err := gocv.VideoCaptureDevice(0)
	if err != nil {
		fmt.Printf("Error opening video capture device: %v\n", err)
		return
	}
	defer webcam.Close()

	window := gocv.NewWindow("Master Tracking System")
	defer window.Close()

	// Main window frame matrix
	img := gocv.NewMat()
	defer img.Close()

	// Initialize our new dedicated Robot Spotter module!
	robotSpotter := NewRobotSpotter()

	// Navigation logic
	//nav := NewNavigator()

	// Mats for Red tracking (HSV)
	hsv := gocv.NewMat()
	defer hsv.Close()
	mask1 := gocv.NewMat()
	defer mask1.Close()
	mask2 := gocv.NewMat()
	defer mask2.Close()
	redMask := gocv.NewMat()
	defer redMask.Close()

	// Mat for Orange Ball tracking (HSV)
	orangeMask := gocv.NewMat()
	defer orangeMask.Close()

	// Mats for White Ball tracking (Grayscale)
	gray := gocv.NewMat()
	defer gray.Close()
	thresh := gocv.NewMat()
	defer thresh.Close()
	kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(5, 5))
	defer kernel.Close()

	// Colors (BGR format)
	blueColor := color.RGBA{255, 0, 0, 0}     // For red zones
	greenColor := color.RGBA{0, 255, 0, 0}    // Safe ball tracking
	yellowColor := color.RGBA{0, 255, 255, 0} // Forbidden touch warning
	cyanColor := color.RGBA{255, 255, 0, 0}   // Navigation arrow

	fmt.Println("System initialized. Tracking red obstacles, white balls, and 1 orange ball simultaneously!")

	for {
		if ok := webcam.Read(&img); !ok || img.Empty() {
			fmt.Println("Device closed or failed to read frame")
			return
		}
		robot := robotSpotter.TrackRobot(&img)

		// ==========================================
		// PART 1: LOCATE ALL RED OBSTACLES & ORANGE
		// ==========================================
		gocv.CvtColor(img, &hsv, gocv.ColorBGRToHSV)

		// Red boundaries
		lowerRed1 := gocv.NewScalar(0, 100, 100, 0)
		upperRed1 := gocv.NewScalar(10, 255, 255, 0)
		lowerRed2 := gocv.NewScalar(170, 100, 100, 0)
		upperRed2 := gocv.NewScalar(180, 255, 255, 0)

		gocv.InRangeWithScalar(hsv, lowerRed1, upperRed1, &mask1)
		gocv.InRangeWithScalar(hsv, lowerRed2, upperRed2, &mask2)
		gocv.BitwiseOr(mask1, mask2, &redMask)

		// Isolate the Orange Ball using HSV (Hue range 11 to 25 for orange)
		lowerOrange := gocv.NewScalar(11, 100, 100, 0)
		upperOrange := gocv.NewScalar(25, 255, 255, 0)
		gocv.InRangeWithScalar(hsv, lowerOrange, upperOrange, &orangeMask)

		redContours := gocv.FindContours(redMask, gocv.RetrievalExternal, gocv.ChainApproxSimple)

		// Dynamic slice to store the positions of red zones
		var redZones []image.Rectangle

		for i := 0; i < redContours.Size(); i++ {
			contour := redContours.At(i)
			if gocv.ContourArea(contour) > 400 {
				rect := gocv.BoundingRect(contour)
				redZones = append(redZones, rect)

				// Draw the blue boundary boxes around red objects
				gocv.Rectangle(&img, rect, blueColor, 2)
			}
		}
		redContours.Close()

		// ==========================================
		// PART 2: LOCATE MULTIPLE BALLS (WHITE & ORANGE)
		// ==========================================
		gocv.CvtColor(img, &gray, gocv.ColorBGRToGray)
		gocv.Threshold(gray, &thresh, 200, 255, gocv.ThresholdBinary)

		// Combine the white binary mask and orange binary mask together!
		gocv.BitwiseOr(thresh, orangeMask, &thresh)

		// Morphological clean up applies to both colors automatically now
		gocv.Dilate(thresh, &thresh, kernel)
		gocv.Erode(thresh, &thresh, kernel)

		ballContours := gocv.FindContours(thresh, gocv.RetrievalExternal, gocv.ChainApproxSimple)

		// Keep track of how many balls have been detected in the frame
		ballsTrackedCount := 0
		anyBallInRedZone := false

		// Collect all detected balls for the navigator
		var balls []Ball

		for i := 0; i < ballContours.Size(); i++ {
			// Stop looking if we already hit our 11 ball maximum target
			if ballsTrackedCount >= 11 {
				break
			}

			contour := ballContours.At(i)
			area := gocv.ContourArea(contour)

			// Using your fine-tuned ball area parameters
			if area > 100 && area < 2000 {
				rect := gocv.BoundingRect(contour)
				aspectRatio := float32(rect.Dx()) / float32(rect.Dy())

				if aspectRatio > 0.7 && aspectRatio < 1.3 {
					// Increment our ball tally
					ballsTrackedCount++

					centerX := rect.Min.X + (rect.Dx() / 2)
					centerY := rect.Min.Y + (rect.Dy() / 2)
					radius := rect.Dx() / 2
					ballCenter := image.Pt(centerX, centerY)

					// ==========================================
					// PART 3: COLLISION/TOUCH DETECTION
					// ==========================================
					ballColor := greenColor // Default to safe green
					frameWidth := img.Cols()
					inRed := false

					// Check if this specific ball falls inside any red zone
					for _, zone := range redZones {
						if zone.Dx() > int(float32(frameWidth)*0.8) {
							continue
						}
						if ballCenter.In(zone) {
							ballColor = yellowColor // Change tracking circle to yellow inside localized red zones
							anyBallInRedZone = true
							inRed = true
							break
						}
					}

					balls = append(balls, Ball{Center: ballCenter, InRedZone: inRed})

					// Draw the individual tracking circle and tracking dot for THIS ball
					gocv.Circle(&img, ballCenter, radius, ballColor, 3)
					gocv.Circle(&img, ballCenter, 4, ballColor, -1)

					// Print the unique coordinates for this ball to the terminal log
					fmt.Printf("[Ball #%d] X: %d, Y: %d\n", ballsTrackedCount, centerX, centerY)
				}
			}
		}
		ballContours.Close()

		// ==========================================
		// PART 4: NAVIGATION
		// ==========================================
		nav := NewNavigator()

		// Link to the physical robot. Set ROBOT_ADDR to the EV3's "ip:port"
		// (e.g. "192.168.1.50:9000"). Leave empty to run in simulation mode.
		robotLink := NewRobotLink(os.Getenv("ROBOT_ADDR"))
		defer robotLink.Close()
		target := PickNextBall(robot, balls)
		var cmd DriveCommand
		if target != nil {
			var navErr error
			cmd, navErr = nav.NextCommand(robot, *target)
			if navErr == nil {
				// Actually drive the robot toward the ball.
				robotLink.Send(cmd)

				if !cmd.Arrived {
					// Draw navigation arrow from robot toward target
					start, end := ArrowPoints(robot.Center, target.Center, 60)
					gocv.Line(&img, start, end, cyanColor, 2)
				}
			} else {
				robotLink.Stop()
			}
		} else {
			// No reachable ball (or robot not detected) — make sure we don't keep moving.
			robotLink.Stop()
		}

		// ==========================================
		// PART 5: DISPLAY SYSTEM GLOBAL STATUS
		// ==========================================
		statusText := fmt.Sprintf("Balls Detected: %d/11", ballsTrackedCount)

		if robot.Detected {
			statusText += fmt.Sprintf(" | Robot: (%d,%d) Heading: %.0f°", robot.Center.X, robot.Center.Y, robot.Angle)
		} else {
			statusText += " | Robot: NOT FOUND"
		}

		globalColor := greenColor
		if anyBallInRedZone {
			statusText += " | Warning: Ball in Red"
			globalColor = yellowColor
		}

		// Navigation debug line
		navText := DebugNavigation(robot, target, cmd)

		// Draw real-time global tracking diagnostics text onto the frame
		gocv.PutText(&img, statusText, image.Pt(20, 40), gocv.FontHersheySimplex, 0.7, globalColor, 2)
		gocv.PutText(&img, navText, image.Pt(20, 70), gocv.FontHersheySimplex, 0.6, cyanColor, 2)

		// Show the master combined tracking view
		window.IMShow(img)

		if window.WaitKey(1) >= 0 {
			break
		}
	}
}
