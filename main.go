package main

import (
	"fmt"
	"image"
	"image/color"
	"math"
	"time"

	"gocv.io/x/gocv"
)

// ballSighting tracks how long a candidate detection has been stable at a position.
type ballSighting struct {
	firstSeen time.Time
	lastSeen  time.Time
	isOrange  bool
}

// stationaryRadius is the pixel tolerance for considering two detections the
// same stationary position across frames.
const stationaryRadius = 15

// phantomDuration is how long we keep driving forward after the ball
// disappears under the harvester.
const phantomDuration = 4000 * time.Millisecond

// phantomThrottle is the forward speed used during the phantom latch burst.
// Set slightly higher than DriveSpeed so the robot pushes fully into the harvester.
const phantomThrottle = 0.55

func main() {
	cfg, err := LoadConfig("config.yml")
	if err != nil {
		fmt.Printf("Error loading config: %v\n", err)
		return
	}

	// stationaryThreshold is how long a detection must remain still before being
	// treated as a real ball.
	const stationaryThreshold = 500 * time.Millisecond

	webcam, err := gocv.VideoCaptureDevice(cfg.Camera.Device)
	if err != nil {
		fmt.Printf("Error opening video capture device: %v\n", err)
		return
	}
	defer webcam.Close()

	window := gocv.NewWindow("Master Tracking System")
	defer window.Close()

	img := gocv.NewMat()
	defer img.Close()

	robotSpotter := NewRobotSpotter()
	goalSpotter := NewGoalSpotter()
	nav := NewNavigator()
	robotLink := NewRobotLink(cfg.Robot.Address)
	defer robotLink.Close()

	state := NewCollectionState()

	// Phantom latch — after the ball disappears under the harvester, we drive
	// straight forward at phantomThrottle for phantomDuration instead of
	// re-running the navigator (which would immediately report Arrived and stop).
	var phantomUntil time.Time // non-zero while latch is active
	var phantomOrange bool     // was the latched ball orange?
	phantomActive := false

	hsv := gocv.NewMat()
	defer hsv.Close()
	mask1 := gocv.NewMat()
	defer mask1.Close()
	mask2 := gocv.NewMat()
	defer mask2.Close()
	redMask := gocv.NewMat()
	defer redMask.Close()

	orangeMask := gocv.NewMat()
	defer orangeMask.Close()

	gray := gocv.NewMat()
	defer gray.Close()
	thresh := gocv.NewMat()
	defer thresh.Close()
	kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(5, 5))
	defer kernel.Close()

	// Colors (BGR format)
	blueColor := color.RGBA{255, 0, 0, 0}
	greenColor := color.RGBA{0, 255, 0, 0}
	yellowColor := color.RGBA{0, 255, 255, 0}
	cyanColor := color.RGBA{255, 255, 0, 0}
	orangeColor := color.RGBA{0, 165, 255, 0}
	magentaColor := color.RGBA{255, 0, 255, 0}
	targetColor := color.RGBA{0, 0, 255, 0}
	grayColor := color.RGBA{160, 160, 160, 0}

	sightings := make(map[image.Point]*ballSighting)

	fmt.Println("System initialised. Running collection FSM.")

	for {
		if ok := webcam.Read(&img); !ok || img.Empty() {
			fmt.Println("Device closed or failed to read frame")
			return
		}

		now := time.Now()

		robot := robotSpotter.TrackRobot(&img)
		goal := goalSpotter.TrackGoal(&img)

		// ==========================================
		// PART 1: RED ZONES & ORANGE MASK
		// ==========================================
		gocv.CvtColor(img, &hsv, gocv.ColorBGRToHSV)

		lowerRed1 := gocv.NewScalar(0, 100, 100, 0)
		upperRed1 := gocv.NewScalar(10, 255, 255, 0)
		lowerRed2 := gocv.NewScalar(170, 100, 100, 0)
		upperRed2 := gocv.NewScalar(180, 255, 255, 0)

		gocv.InRangeWithScalar(hsv, lowerRed1, upperRed1, &mask1)
		gocv.InRangeWithScalar(hsv, lowerRed2, upperRed2, &mask2)
		gocv.BitwiseOr(mask1, mask2, &redMask)

		lowerOrange := gocv.NewScalar(11, 100, 100, 0)
		upperOrange := gocv.NewScalar(25, 255, 255, 0)
		gocv.InRangeWithScalar(hsv, lowerOrange, upperOrange, &orangeMask)

		redContours := gocv.FindContours(redMask, gocv.RetrievalExternal, gocv.ChainApproxSimple)
		var redZones []image.Rectangle
		for i := 0; i < redContours.Size(); i++ {
			contour := redContours.At(i)
			if gocv.ContourArea(contour) > 400 {
				rect := gocv.BoundingRect(contour)
				redZones = append(redZones, rect)
				gocv.Rectangle(&img, rect, blueColor, 1)
			}
		}
		redContours.Close()

		// ==========================================
		// PART 2: BALL DETECTION
		// ==========================================
		gocv.CvtColor(img, &gray, gocv.ColorBGRToGray)
		gocv.Threshold(gray, &thresh, 180, 255, gocv.ThresholdBinary)
		gocv.BitwiseOr(thresh, orangeMask, &thresh)
		gocv.Dilate(thresh, &thresh, kernel)
		gocv.Erode(thresh, &thresh, kernel)

		ballContours := gocv.FindContours(thresh, gocv.RetrievalExternal, gocv.ChainApproxSimple)

		seenKeys := make(map[image.Point]bool)
		ballsTrackedCount := 0
		anyBallInRedZone := false
		var balls []Ball

		for i := 0; i < ballContours.Size(); i++ {
			contour := ballContours.At(i)
			area := gocv.ContourArea(contour)

			if area > 100 && area < 300 {
				rect := gocv.BoundingRect(contour)
				aspectRatio := float32(rect.Dx()) / float32(rect.Dy())

				if aspectRatio > 0.7 && aspectRatio < 1.5 {
					centerX := rect.Min.X + (rect.Dx() / 2)
					centerY := rect.Min.Y + (rect.Dy() / 2)
					ballCenter := image.Pt(centerX, centerY)

					if robot.Detected && ballCenter.In(robot.Box) {
						continue
					}
					if goal.Detected && ballCenter.In(goal.Box) {
						continue
					}

					radius := rect.Dx() / 2

					isOrange := false
					if centerX >= 0 && centerX < orangeMask.Cols() &&
						centerY >= 0 && centerY < orangeMask.Rows() {
						if orangeMask.GetUCharAt(centerY, centerX) > 0 {
							isOrange = true
						}
					}

					var matchKey *image.Point
					for k := range sightings {
						dx := float64(k.X - centerX)
						dy := float64(k.Y - centerY)
						if math.Sqrt(dx*dx+dy*dy) <= stationaryRadius {
							k := k
							matchKey = &k
							break
						}
					}

					var key image.Point
					if matchKey != nil {
						key = *matchKey
						sightings[key].lastSeen = now
						sightings[key].isOrange = isOrange
					} else {
						key = ballCenter
						sightings[key] = &ballSighting{firstSeen: now, lastSeen: now, isOrange: isOrange}
					}
					seenKeys[key] = true

					dwellTime := now.Sub(sightings[key].firstSeen)
					confirmed := dwellTime >= stationaryThreshold

					if confirmed && ballsTrackedCount < 11 {
						ballsTrackedCount++

						frameWidth := img.Cols()
						inRed := false
						for _, zone := range redZones {
							if zone.Dx() > int(float32(frameWidth)*0.8) {
								continue
							}
							if ballCenter.In(zone) {
								anyBallInRedZone = true
								inRed = true
								break
							}
						}

						balls = append(balls, Ball{Center: ballCenter, InRedZone: inRed, IsOrange: isOrange})

						drawColor := greenColor
						if inRed {
							drawColor = yellowColor
						} else if isOrange {
							drawColor = orangeColor
						}
						gocv.Circle(&img, ballCenter, radius, drawColor, 1)
						gocv.Circle(&img, ballCenter, 4, drawColor, -1)

						fmt.Printf("[Ball #%d] X: %d, Y: %d orange=%v\n", ballsTrackedCount, centerX, centerY, isOrange)
					} else if !confirmed {
						remaining := stationaryThreshold - dwellTime
						gocv.Circle(&img, ballCenter, radius, grayColor, 1)
						gocv.PutText(&img,
							fmt.Sprintf("%.1fs", remaining.Seconds()),
							image.Pt(ballCenter.X+radius+2, ballCenter.Y+4),
							gocv.FontHersheySimplex, 0.35, grayColor, 1)
					}
				}
			}
		}
		ballContours.Close()

		for k := range sightings {
			if !seenKeys[k] {
				delete(sightings, k)
			}
		}

		// ==========================================
		// PART 4: COLLECTION STATE MACHINE
		// ==========================================
		var cmd DriveCommand
		var navTarget *Ball

		switch state.Phase {

		case PhasePickBall:
			if state.BallsCollected >= state.TotalBalls {
				state.Phase = PhaseDone
				break
			}

			// ---- PHANTOM LATCH CHECK ----
			// Drive straight forward at a fixed throttle for phantomDuration.
			// We do NOT re-invoke the navigator here — the robot is already past
			// ArrivedRadius so the navigator would return Arrived immediately and
			// produce zero throttle, stalling the pickup.
			if phantomActive {
				if now.After(phantomUntil) {
					// Latch expired — ball should be inside harvester now.
					fmt.Printf("[FSM] Phantom latch expired. Ball collected (orange=%v). Delivering to goal.\n", phantomOrange)
					if phantomOrange {
						state.CarryingOrange = true
					}
					phantomActive = false
					state.Phase = PhaseDeliverGoal
					break
				}
				// Still within latch window — push straight forward, no steering.
				remaining := time.Until(phantomUntil)
				cmd = DriveCommand{Throttle: phantomThrottle, Turn: 0, Arrived: false}
				robotLink.Send(cmd)
				gocv.PutText(&img,
					fmt.Sprintf("PHANTOM %.1fs", remaining.Seconds()),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, targetColor, 2)
				break
			}

			// ---- NORMAL PICK LOGIC ----
			target := PickNextBall(robot, balls, state.OrangeDelivered)
			navTarget = target

			if target == nil {
				robotLink.Stop()
				break
			}

			var navErr error
			cmd, navErr = nav.NextCommand(robot, *target)
			if navErr != nil {
				robotLink.Stop()
				break
			}

			if cmd.Arrived {
				// Ball reached ArrivedRadius — start phantom latch.
				// The robot will drive straight forward for phantomDuration
				// to push the ball fully into the harvester.
				phantomActive = true
				phantomUntil = now.Add(phantomDuration)
				phantomOrange = target.IsOrange
				fmt.Printf("[FSM] Arrived at ball (orange=%v). Starting %.0fms straight-drive phantom latch.\n",
					phantomOrange, float64(phantomDuration.Milliseconds()))
			} else {
				robotLink.Send(cmd)
				start, end := ArrowPoints(robot.Center, target.Center, 60)
				gocv.Line(&img, start, end, cyanColor, 1)
			}

		case PhaseDeliverGoal:
			if !goal.Detected {
				robotLink.Stop()
				gocv.PutText(&img, "WAITING FOR GOAL MARKER", image.Pt(20, 100),
					gocv.FontHersheySimplex, 0.7, magentaColor, 1)
				break
			}

			var navErr error
			cmd, navErr = nav.NextCommandToPoint(robot, goal.Center)
			if navErr != nil {
				robotLink.Stop()
				break
			}

			if cmd.Arrived {
				state.BallsCollected++
				if state.CarryingOrange {
					state.OrangeDelivered = true
					state.CarryingOrange = false
				}
				fmt.Printf("[FSM] Delivered. Total: %d/%d\n", state.BallsCollected, state.TotalBalls)

				if state.BallsCollected >= state.TotalBalls {
					state.Phase = PhaseDone
					fmt.Println("[FSM] All balls delivered! Stopping.")
				} else {
					state.Phase = PhasePickBall
				}
			} else {
				robotLink.Send(cmd)
				start, end := ArrowPoints(robot.Center, goal.Center, 60)
				gocv.Line(&img, start, end, magentaColor, 1)
			}

		case PhaseDone:
			robotLink.Stop()
		}

		// ==========================================
		// PART 5: DEBUG — HIGHLIGHT TARGETED BALL
		// ==========================================
		if navTarget != nil {
			targetRadius := 14
			gocv.Circle(&img, navTarget.Center, targetRadius, targetColor, 1)
			gocv.Circle(&img, navTarget.Center, 5, targetColor, -1)
		}

		// ==========================================
		// PART 6: DISPLAY SYSTEM GLOBAL STATUS
		// ==========================================
		phaseStr := map[Phase]string{
			PhasePickBall:    "PICK",
			PhaseDeliverGoal: "DELIVER",
			PhaseDone:        "DONE",
		}[state.Phase]

		statusText := fmt.Sprintf("Balls: %d/11 | Phase: %s | Delivered: %d",
			ballsTrackedCount, phaseStr, state.BallsCollected)

		if robot.Detected {
			statusText += fmt.Sprintf(" | Robot: (%d,%d) %.0f°",
				robot.Center.X, robot.Center.Y, robot.Angle)
		} else {
			statusText += " | Robot: NOT FOUND"
		}
		if goal.Detected {
			statusText += fmt.Sprintf(" | Goal: (%d,%d)", goal.Center.X, goal.Center.Y)
		} else {
			statusText += " | Goal: NOT FOUND"
		}

		globalColor := greenColor
		if anyBallInRedZone {
			statusText += " | Ball in Red Zone"
			globalColor = yellowColor
		}
		if state.Phase == PhaseDone {
			globalColor = magentaColor
		}

		navText := DebugNavigation(robot, navTarget, cmd)

		gocv.PutText(&img, statusText, image.Pt(20, 40), gocv.FontHersheySimplex, 0.6, globalColor, 1)
		gocv.PutText(&img, navText, image.Pt(20, 70), gocv.FontHersheySimplex, 0.55, cyanColor, 1)

		window.IMShow(img)
		if window.WaitKey(1) >= 0 {
			break
		}
	}
}
