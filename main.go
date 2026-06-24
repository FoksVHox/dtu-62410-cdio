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
}

// stationaryRadius is the pixel tolerance for considering two detections the
// same stationary position across frames.
const stationaryRadius = 15

// phantomDuration is how long we keep driving forward after the ball
// disappears under the harvester.
const phantomDuration = 50 * time.Millisecond

// phantomThrottle is the forward speed used during the phantom latch burst.
const phantomThrottle = 0.55

// ── Goal-delivery tuning ──────────────────────────────────────────────────────

// deliverApproachOffsetPx is the distance (pixels) in front of the goal marker
// that the robot drives to before spinning and reversing in.
// "In front" means along the marker's outward face normal.
const deliverApproachOffsetPx = 150.0

// deliverApproachArrivalPx is the radius (pixels) at which the navigator
// considers the staging / approach point reached.
const deliverApproachArrivalPx = 35.0

// deliverTurn180Tol is the heading error (degrees) at which the perpendicular
// alignment to the goal marker face is considered complete.
const deliverTurn180Tol = 8.0

// deliverBackupSpeed is the reverse throttle magnitude used when backing into
// the goal.
const deliverBackupSpeed = 0.40

// deliverGoalArrivalPx is the pixel distance from the goal centre at which we
// consider the robot close enough to open the latch.
// 120px leaves comfortable room for the latch to open.
const deliverGoalArrivalPx = 120.0

// deliverLatchOpenDuration is how long the latch stays open before closing.
const deliverLatchOpenDuration = 4 * time.Second

// deliverLatchCloseDuration is how long we wait after LATCH_CLOSE before
// the FSM moves on.
const deliverLatchCloseDuration = 4 * time.Second

func main() {
	cfg, err := LoadConfig("config.yml")
	if err != nil {
		fmt.Printf("Error loading config: %v\n", err)
		return
	}

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

	var phantomUntil time.Time
	phantomActive := false
	var deliverTimer time.Time
	var lockedTarget *Ball

	// Cache the most recent valid goal face angle + staging point.
	var lastGoalFaceAngle float64
	var lastGoalFaceAngleValid bool
	var lastStagingPoint image.Point

	hsv := gocv.NewMat()
	defer hsv.Close()
	mask1 := gocv.NewMat()
	defer mask1.Close()
	mask2 := gocv.NewMat()
	defer mask2.Close()
	redMask := gocv.NewMat()
	defer redMask.Close()

	gray := gocv.NewMat()
	defer gray.Close()
	thresh := gocv.NewMat()
	defer thresh.Close()
	kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(5, 5))
	defer kernel.Close()

	blueColor := color.RGBA{255, 0, 0, 0}
	greenColor := color.RGBA{0, 255, 0, 0}
	yellowColor := color.RGBA{0, 255, 255, 0}
	cyanColor := color.RGBA{255, 255, 0, 0}
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

		// Cache goal face angle and recompute staging point whenever the marker is visible.
		if goal.Detected {
			lastGoalFaceAngle = goal.FaceAngle
			lastGoalFaceAngleValid = true
			// Staging point: goal.Center + deliverApproachOffsetPx along the face normal.
			faceRad := goal.FaceAngle * math.Pi / 180.0
			lastStagingPoint = image.Pt(
				goal.Center.X+int(math.Cos(faceRad)*deliverApproachOffsetPx),
				goal.Center.Y+int(math.Sin(faceRad)*deliverApproachOffsetPx),
			)
		}

		// ── PART 1: RED ZONES ────────────────────────────────────────────────────
		gocv.CvtColor(img, &hsv, gocv.ColorBGRToHSV)

		lowerRed1 := gocv.NewScalar(0, 100, 100, 0)
		upperRed1 := gocv.NewScalar(10, 255, 255, 0)
		lowerRed2 := gocv.NewScalar(170, 100, 100, 0)
		upperRed2 := gocv.NewScalar(180, 255, 255, 0)

		gocv.InRangeWithScalar(hsv, lowerRed1, upperRed1, &mask1)
		gocv.InRangeWithScalar(hsv, lowerRed2, upperRed2, &mask2)
		gocv.BitwiseOr(mask1, mask2, &redMask)

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

		// ── PART 2: BALL DETECTION ───────────────────────────────────────────────
		gocv.CvtColor(img, &gray, gocv.ColorBGRToGray)
		gocv.Threshold(gray, &thresh, 180, 255, gocv.ThresholdBinary)
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

					if centerX >= 0 && centerX < hsv.Cols() &&
						centerY >= 0 && centerY < hsv.Rows() {
						h := hsv.GetUCharAt(centerY, centerX*3)
						s := hsv.GetUCharAt(centerY, centerX*3+1)
						if h >= 11 && h <= 25 && s > 100 {
							continue
						}
					}

					radius := rect.Dx() / 2

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
					} else {
						key = ballCenter
						sightings[key] = &ballSighting{firstSeen: now, lastSeen: now}
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

						balls = append(balls, Ball{Center: ballCenter, InRedZone: inRed})

						drawColor := greenColor
						if inRed {
							drawColor = yellowColor
						}
						gocv.Circle(&img, ballCenter, radius, drawColor, 1)
						gocv.Circle(&img, ballCenter, 4, drawColor, -1)

						fmt.Printf("[Ball #%d] X: %d, Y: %d\n", ballsTrackedCount, centerX, centerY)
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

		// ── PART 4: COLLECTION STATE MACHINE ────────────────────────────────────
		var cmd DriveCommand
		var navTarget *Ball

		switch state.Phase {

		case PhasePickBall:
			if state.BallsCollected >= state.TotalBalls {
				state.Phase = PhaseDone
				break
			}

			// ── PHANTOM LATCH CHECK ──────────────────────────────────────────────
			if phantomActive {
				if now.After(phantomUntil) {
					state.BallsInHarvester++
					fmt.Printf("[FSM] Phantom latch expired. Harvester: %d/%d balls.\n",
						state.BallsInHarvester, state.MaxHarvesterLoad)
					phantomActive = false
					lockedTarget = nil
					robotLink.Stop()

					remainingOnField := state.TotalBalls - state.BallsCollected - state.BallsInHarvester
					shouldDeliver := state.BallsInHarvester >= state.MaxHarvesterLoad ||
						remainingOnField <= 0
					if shouldDeliver {
						fmt.Printf("[FSM] Harvester full (%d/%d) or no balls left — delivering to goal.\n",
							state.BallsInHarvester, state.MaxHarvesterLoad)
						state.DelivSubPhase = DelivSubApproach
						nav = NewNavigator()
						state.Phase = PhaseDeliverGoal
					} else {
						nav = NewNavigator()
					}
					break
				}
				robotLink.ForceThrottle(phantomThrottle)
				remaining := time.Until(phantomUntil)
				gocv.PutText(&img,
					fmt.Sprintf("PHANTOM %.1fs", remaining.Seconds()),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, targetColor, 2)
				break
			}

			// ── BALL SELECTION ───────────────────────────────────────────────────
			if lockedTarget == nil {
				lockedTarget = PickNextBall(robot, balls)
				if lockedTarget != nil {
					fmt.Printf("[FSM] Locked onto ball at (%d,%d)\n",
						lockedTarget.Center.X, lockedTarget.Center.Y)
					nav = NewNavigator()
				}
			} else {
				const lockMatchPx = 30.0
				var refreshed *Ball
				for i := range balls {
					dx := float64(balls[i].Center.X - lockedTarget.Center.X)
					dy := float64(balls[i].Center.Y - lockedTarget.Center.Y)
					if math.Sqrt(dx*dx+dy*dy) <= lockMatchPx {
						refreshed = &balls[i]
						break
					}
				}
				if refreshed != nil {
					lockedTarget = refreshed
				}
			}

			navTarget = lockedTarget

			if navTarget == nil {
				if state.BallsInHarvester > 0 {
					fmt.Printf("[FSM] No balls visible; %d in harvester — delivering to goal.\n",
						state.BallsInHarvester)
					state.DelivSubPhase = DelivSubApproach
					nav = NewNavigator()
					state.Phase = PhaseDeliverGoal
				} else {
					robotLink.Stop()
				}
				break
			}

			var navErr error
			cmd, navErr = nav.NextCommand(robot, *navTarget)
			if navErr != nil {
				robotLink.Stop()
				break
			}

			if cmd.Arrived {
				phantomActive = true
				phantomUntil = now.Add(phantomDuration)
				fmt.Printf("[FSM] Arrived at ball. Starting %.0fms phantom latch.\n",
					float64(phantomDuration.Milliseconds()))
				robotLink.ForceThrottle(phantomThrottle)
			} else {
				robotLink.Send(cmd)
				start, end := ArrowPoints(robot.Center, navTarget.Center, 60)
				gocv.Line(&img, start, end, cyanColor, 1)
			}

		// ── GOAL DELIVERY — 7-STEP SUB-FSM ──────────────────────────────────────
		// APPROACH → TURN180 → BACK_UP → OPEN_LATCH → WAIT_LATCH → CLOSE_LATCH → WAIT_CLOSE
		case PhaseDeliverGoal:
			if !robot.Detected {
				robotLink.Stop()
				gocv.PutText(&img, "DELIVER: robot not found", image.Pt(20, 100),
					gocv.FontHersheySimplex, 0.6, magentaColor, 2)
				break
			}

			switch state.DelivSubPhase {

			// ── Step 0: Drive forward to staging point in front of the goal ───────
			//
			// We project a point deliverApproachOffsetPx pixels out from the goal
			// centre along its face normal. The normal navigator drives the robot
			// there. Once arrived the robot is in front of the marker and can spin
			// cleanly before reversing straight in.
			case DelivSubApproach:
				if !lastGoalFaceAngleValid {
					// No marker data yet — wait in place.
					robotLink.Stop()
					gocv.PutText(&img, "DELIVER APPROACH: waiting for goal marker",
						image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
					break
				}

				// Draw the staging point on the overlay.
				gocv.Circle(&img, lastStagingPoint, 8, magentaColor, -1)
				gocv.Line(&img, goal.Center, lastStagingPoint, magentaColor, 1)

				// Use a custom arrival radius so we don't overshoot.
				nav.GoalArrivedRadius = deliverApproachArrivalPx
				approachCmd, navErr := nav.NextCommandToPoint(robot, lastStagingPoint)
				if navErr != nil {
					robotLink.Stop()
					break
				}

				dx := float64(lastStagingPoint.X - robot.Center.X)
				dy := float64(lastStagingPoint.Y - robot.Center.Y)
				dist := math.Sqrt(dx*dx + dy*dy)

				gocv.PutText(&img,
					fmt.Sprintf("DELIVER: APPROACH dist=%.0fpx staging=(%d,%d)",
						dist, lastStagingPoint.X, lastStagingPoint.Y),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)

				if approachCmd.Arrived {
					fmt.Println("[DELIVER] APPROACH complete — robot at staging point. Switching to TURN180.")
					robotLink.Stop()
					nav = NewNavigator() // reset navigator for next use
					state.DelivSubPhase = DelivSubTurn180
				} else {
					robotLink.Send(approachCmd)
					start, end := ArrowPoints(robot.Center, lastStagingPoint, 60)
					gocv.Line(&img, start, end, magentaColor, 1)
				}

			// ── Step 1: Spin to align robot back with the marker face normal ──────
			case DelivSubTurn180:
				var targetHeading float64
				if goal.Detected {
					targetHeading = math.Mod(goal.FaceAngle+180, 360)
					fmt.Printf("[DELIVER] TURN180 | FaceAngle=%.1f° → targetHeading=%.1f°\n",
						goal.FaceAngle, targetHeading)
				} else if lastGoalFaceAngleValid {
					targetHeading = math.Mod(lastGoalFaceAngle+180, 360)
					gocv.PutText(&img, "DELIVER TURN180: using cached angle",
						image.Pt(20, 120), gocv.FontHersheySimplex, 0.5, magentaColor, 1)
				} else {
					robotLink.Stop()
					gocv.PutText(&img, "DELIVER TURN180: waiting for goal marker",
						image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
					break
				}

				headingErr := normaliseAngle(targetHeading - robot.Angle)
				absErr := math.Abs(headingErr)

				gocv.PutText(&img,
					fmt.Sprintf("DELIVER: TURN180 err=%.0f° target=%.0f°", headingErr, targetHeading),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)

				if absErr <= deliverTurn180Tol {
					fmt.Println("[DELIVER] TURN180 complete — perpendicular to goal. Switching to BACK_UP.")
					robotLink.Stop()
					state.DelivSubPhase = DelivSubBackUp
					break
				}

				turnSign := math.Copysign(1, headingErr)
				turnMag := math.Min(absErr/15.0, 1.0) * 0.4
				robotLink.Send(DriveCommand{Throttle: 0, Turn: turnSign * turnMag})

			// ── Step 2: Reverse straight into the goal, stop short ────────────────
			case DelivSubBackUp:
				if !goal.Detected {
					robotLink.ForceReverse(deliverBackupSpeed)
					gocv.PutText(&img, "DELIVER BACK_UP: goal marker lost — continuing",
						image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
					break
				}

				dx := float64(goal.Center.X - robot.Center.X)
				dy := float64(goal.Center.Y - robot.Center.Y)
				dist := math.Sqrt(dx*dx + dy*dy)

				gocv.PutText(&img,
					fmt.Sprintf("DELIVER: BACK_UP dist=%.0fpx (stop at %.0fpx)", dist, deliverGoalArrivalPx),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
				gocv.Line(&img, robot.Center, goal.Center, magentaColor, 1)
				fmt.Printf("[DELIVER] BACK_UP | dist=%.1fpx\n", dist)

				if dist <= deliverGoalArrivalPx {
					fmt.Printf("[DELIVER] BACK_UP complete — stopped %.0fpx from marker. Opening latch.\n", dist)
					robotLink.Stop()
					state.DelivSubPhase = DelivSubOpenLatch
					break
				}

				var approachAngle float64
				if goal.Detected {
					approachAngle = math.Mod(goal.FaceAngle+180, 360)
				} else {
					approachAngle = math.Mod(lastGoalFaceAngle+180, 360)
				}
				steerErr := normaliseAngle(approachAngle - robot.Angle)
				steerCorr := math.Max(-0.25, math.Min(0.25, steerErr*0.015))
				robotLink.Send(DriveCommand{Throttle: -deliverBackupSpeed, Turn: steerCorr})

			// ── Step 3: Send LATCH_OPEN ───────────────────────────────────────────
			case DelivSubOpenLatch:
				fmt.Printf("[DELIVER] Sending LATCH_OPEN (releasing %d ball(s))\n",
					state.BallsInHarvester)
				robotLink.SendLatchOpen()
				deliverTimer = now.Add(deliverLatchOpenDuration)
				state.DelivSubPhase = DelivSubWaitLatch

			// ── Step 4: Wait for balls to roll out ────────────────────────────────
			case DelivSubWaitLatch:
				remaining := time.Until(deliverTimer)
				gocv.PutText(&img,
					fmt.Sprintf("DELIVER: LATCH OPEN %.1fs (%d balls)", remaining.Seconds(),
						state.BallsInHarvester),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
				if now.After(deliverTimer) {
					fmt.Println("[DELIVER] Open timer expired — sending LATCH_CLOSE")
					state.DelivSubPhase = DelivSubCloseLatch
				}

			// ── Step 5: Send LATCH_CLOSE ──────────────────────────────────────────
			case DelivSubCloseLatch:
				fmt.Println("[DELIVER] Sending LATCH_CLOSE to EV3.")
				robotLink.SendLatchClose()
				deliverTimer = now.Add(deliverLatchCloseDuration)
				state.DelivSubPhase = DelivSubWaitClose

			// ── Step 6: Wait for latch to retract ─────────────────────────────────
			case DelivSubWaitClose:
				remaining := time.Until(deliverTimer)
				gocv.PutText(&img,
					fmt.Sprintf("DELIVER: LATCH CLOSING %.1fs", remaining.Seconds()),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
				if now.After(deliverTimer) {
					state.BallsCollected += state.BallsInHarvester
					state.BallsInHarvester = 0
					fmt.Printf("[DELIVER] Latch closed. Total delivered: %d/%d\n",
						state.BallsCollected, state.TotalBalls)
					state.DelivSubPhase = DelivSubApproach
					if state.BallsCollected >= state.TotalBalls {
						state.Phase = PhaseDone
						fmt.Println("[FSM] All balls delivered! Stopping.")
					} else {
						state.Phase = PhasePickBall
						nav = NewNavigator()
					}
				}
			}

		case PhaseDone:
			robotLink.Stop()
		}

		// ── PART 5: HIGHLIGHT TARGETED BALL ─────────────────────────────────────
		if navTarget != nil {
			gocv.Circle(&img, navTarget.Center, 14, targetColor, 1)
			gocv.Circle(&img, navTarget.Center, 5, targetColor, -1)
		}

		// ── PART 6: STATUS HUD ───────────────────────────────────────────────────
		phaseStr := map[Phase]string{
			PhasePickBall:    "PICK",
			PhaseDeliverGoal: "DELIVER",
			PhaseDone:        "DONE",
		}[state.Phase]

		statusText := fmt.Sprintf("Balls: %d/11 | Phase: %s | Delivered: %d | Harvester: %d/%d",
			ballsTrackedCount, phaseStr, state.BallsCollected,
			state.BallsInHarvester, state.MaxHarvesterLoad)

		if robot.Detected {
			statusText += fmt.Sprintf(" | Robot: (%d,%d) %.0f°",
				robot.Center.X, robot.Center.Y, robot.Angle)
		} else {
			statusText += " | Robot: NOT FOUND"
		}
		if goal.Detected {
			statusText += fmt.Sprintf(" | Goal: (%d,%d) face=%.0f°", goal.Center.X, goal.Center.Y, goal.FaceAngle)
			if state.Phase == PhaseDeliverGoal && state.DelivSubPhase == DelivSubApproach {
				statusText += fmt.Sprintf(" staging=(%d,%d)", lastStagingPoint.X, lastStagingPoint.Y)
			}
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
