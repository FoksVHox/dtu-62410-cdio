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
// Set slightly higher than DriveSpeed so the robot pushes fully into the harvester.
const phantomThrottle = 0.55

// ── Goal-delivery tuning ──────────────────────────────────────────────────────

// deliverTurn180Tol is the heading error (degrees) at which the perpendicular
// alignment to the goal marker face is considered complete.
const deliverTurn180Tol = 8.0

// deliverBackupSpeed is the reverse throttle magnitude used when backing into
// the goal (positive value; sign is applied inside ForceReverse).
const deliverBackupSpeed = 0.40

// deliverGoalArrivalPx is the pixel distance from the goal centre at which we
// consider the robot close enough to open the latch.
// Raised to 120px so the robot stops comfortably before the marker.
const deliverGoalArrivalPx = 120.0

// deliverLatchOpenDuration is how long the latch stays open (motor forward)
// before the close command is sent.
const deliverLatchOpenDuration = 4 * time.Second

// deliverLatchCloseDuration is how long we wait after sending LATCH_CLOSE for
// the back motor to fully retract the latch before the FSM moves on.
const deliverLatchCloseDuration = 4 * time.Second

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
	phantomActive := false

	// deliverTimer is reused for both the latch-open wait and the latch-close wait.
	var deliverTimer time.Time

	// lockedTarget is the ball the robot is currently committed to collecting.
	var lockedTarget *Ball

	// lastGoalFaceAngle stores the most recent valid goal face angle so the
	// alignment step can continue even if the marker is briefly occluded.
	var lastGoalFaceAngle float64
	var lastGoalFaceAngleValid bool

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

	// Colors (BGR format)
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

		// Cache the goal face angle whenever we have a valid reading.
		if goal.Detected {
			lastGoalFaceAngle = goal.FaceAngle
			lastGoalFaceAngleValid = true
		}

		// ==========================================
		// PART 1: RED ZONES
		// ==========================================
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

		// ==========================================
		// PART 2: BALL DETECTION (white balls only)
		// The orange VIP ball is intentionally ignored.
		// ==========================================
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

					// Skip orange-coloured detections — ignore the VIP ball entirely.
					if centerX >= 0 && centerX < hsv.Cols() &&
						centerY >= 0 && centerY < hsv.Rows() {
						h := hsv.GetUCharAt(centerY, centerX*3)
						s := hsv.GetUCharAt(centerY, centerX*3+1)
						if h >= 11 && h <= 25 && s > 100 {
							continue // orange hue — skip
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
			if phantomActive {
				if now.After(phantomUntil) {
					// Ball is now fully harvested. Count it.
					state.BallsInHarvester++
					fmt.Printf("[FSM] Phantom latch expired. Harvester: %d/%d balls.\n",
						state.BallsInHarvester, state.MaxHarvesterLoad)
					phantomActive = false
					lockedTarget = nil
					robotLink.Stop()

					// Decide: go get more balls, or deliver the current batch?
					remainingOnField := state.TotalBalls - state.BallsCollected - state.BallsInHarvester
					shouldDeliver := state.BallsInHarvester >= state.MaxHarvesterLoad ||
						remainingOnField <= 0
					if shouldDeliver {
						fmt.Printf("[FSM] Harvester full (%d/%d) or no balls left — delivering to goal.\n",
							state.BallsInHarvester, state.MaxHarvesterLoad)
						state.DelivSubPhase = DelivSubTurn180
						nav = NewNavigator()
						state.Phase = PhaseDeliverGoal
					} else {
						// Keep collecting — reset nav to go after the next ball.
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

			// ---- BALL SELECTION: pick once and lock ----
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
				// No ball visible on the field.
				if state.BallsInHarvester > 0 {
					// We're already carrying some — go deliver them.
					fmt.Printf("[FSM] No balls visible; %d in harvester — delivering to goal.\n",
						state.BallsInHarvester)
					state.DelivSubPhase = DelivSubTurn180
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
				fmt.Printf("[FSM] Arrived at ball. Starting %.0fms straight-drive phantom latch.\n",
					float64(phantomDuration.Milliseconds()))
				robotLink.ForceThrottle(phantomThrottle)
			} else {
				robotLink.Send(cmd)
				start, end := ArrowPoints(robot.Center, navTarget.Center, 60)
				gocv.Line(&img, start, end, cyanColor, 1)
			}

		// ==========================================
		// GOAL DELIVERY — 6-STEP SUB-FSM
		// TURN180 → BACK_UP → OPEN_LATCH → WAIT_LATCH → CLOSE_LATCH → WAIT_CLOSE
		// ==========================================
		case PhaseDeliverGoal:
			if !robot.Detected {
				robotLink.Stop()
				gocv.PutText(&img, "DELIVER: robot not found", image.Pt(20, 100),
					gocv.FontHersheySimplex, 0.6, magentaColor, 2)
				break
			}

			switch state.DelivSubPhase {

			// ── Step 1: Align robot back perpendicular to the goal marker face ───────
			//
			// Strategy: read FaceAngle from the ArUco marker (the outward normal of
			// its top edge). The robot must reverse along that normal, so its back
			// must face the direction the marker is "looking" at us, i.e. the robot
			// heading = FaceAngle + 180°.
			//
			// If the marker is briefly occluded we fall back to the last cached
			// FaceAngle, and as a last resort to the bearing-to-centre method.
			case DelivSubTurn180:
				// Determine target heading: robot back must face the marker's outward normal.
				var targetHeading float64
				if goal.Detected {
					// Primary: use the marker face normal directly.
					// The robot's FRONT must face away from the marker (FaceAngle + 180),
					// so that the robot's BACK faces the marker front — ready to reverse in.
					targetHeading = math.Mod(goal.FaceAngle+180, 360)
					fmt.Printf("[DELIVER] TURN180 | using marker FaceAngle=%.1f° → targetHeading=%.1f°\n",
						goal.FaceAngle, targetHeading)
				} else if lastGoalFaceAngleValid {
					// Fallback 1: use cached face angle from last frame.
					targetHeading = math.Mod(lastGoalFaceAngle+180, 360)
					gocv.PutText(&img, "DELIVER TURN180: marker occluded — using cached angle",
						image.Pt(20, 120), gocv.FontHersheySimplex, 0.5, magentaColor, 1)
					fmt.Printf("[DELIVER] TURN180 | marker lost — using cached FaceAngle=%.1f° → targetHeading=%.1f°\n",
						lastGoalFaceAngle, targetHeading)
				} else {
					// Fallback 2: no marker data at all — wait in place.
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
					fmt.Println("[DELIVER] TURN180 complete — robot aligned perpendicular to goal. Switching to BACK_UP.")
					robotLink.Stop()
					state.DelivSubPhase = DelivSubBackUp
					break
				}

				turnSign := math.Copysign(1, headingErr)
				turnMag := math.Min(absErr/15.0, 1.0) * 0.4
				robotLink.Send(DriveCommand{Throttle: 0, Turn: turnSign * turnMag})

			// ── Step 2: Reverse straight into the goal, stop short of the marker ────
			//
			// The robot reverses along the marker face normal. Steering correction
			// keeps the approach straight. We stop at deliverGoalArrivalPx pixels
			// from the marker centre, which leaves enough room to open the latch.
			case DelivSubBackUp:
				if !goal.Detected {
					// Marker briefly lost — keep reversing on the last known heading.
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
				fmt.Printf("[DELIVER] BACK_UP | dist=%.1fpx goal=(%d,%d) robot=(%d,%d)\n",
					dist, goal.Center.X, goal.Center.Y, robot.Center.X, robot.Center.Y)

				if dist <= deliverGoalArrivalPx {
					fmt.Printf("[DELIVER] BACK_UP complete — stopped %.0fpx from goal marker. Opening latch.\n", dist)
					robotLink.Stop()
					state.DelivSubPhase = DelivSubOpenLatch
					break
				}

				// Steering: keep the robot aligned with the marker face normal while reversing.
				// Use the live FaceAngle when available, otherwise fall back to cached value.
				var approachAngle float64
				if goal.Detected {
					approachAngle = math.Mod(goal.FaceAngle+180, 360)
				} else {
					approachAngle = math.Mod(lastGoalFaceAngle+180, 360)
				}
				// The robot's back should face approachAngle, i.e. robot.Angle = approachAngle.
				// Steer to correct drift.
				steerErr := normaliseAngle(approachAngle - robot.Angle)
				steerCorr := math.Max(-0.25, math.Min(0.25, steerErr*0.015))
				robotLink.Send(DriveCommand{Throttle: -deliverBackupSpeed, Turn: steerCorr})

			// ── Step 3: Send LATCH_OPEN, start open-wait timer ───────────────────
			case DelivSubOpenLatch:
				fmt.Printf("[DELIVER] Sending LATCH_OPEN to EV3 (releasing %d ball(s))\n",
					state.BallsInHarvester)
				robotLink.SendLatchOpen()
				deliverTimer = now.Add(deliverLatchOpenDuration)
				state.DelivSubPhase = DelivSubWaitLatch

			// ── Step 4: Hold still while balls roll out ───────────────────────────
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
				// Robot stays stationary; no drive command sent.

			// ── Step 5: Send LATCH_CLOSE once, start close-wait timer ────────────
			case DelivSubCloseLatch:
				fmt.Println("[DELIVER] Sending LATCH_CLOSE to EV3 (back motor reverses).")
				robotLink.SendLatchClose()
				deliverTimer = now.Add(deliverLatchCloseDuration)
				// Advance immediately so the next frame enters DelivSubWaitClose.
				state.DelivSubPhase = DelivSubWaitClose

			// ── Step 6: Wait for the back motor to finish retracting ────────────
			case DelivSubWaitClose:
				remaining := time.Until(deliverTimer)
				gocv.PutText(&img,
					fmt.Sprintf("DELIVER: LATCH CLOSING %.1fs", remaining.Seconds()),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
				// Robot stays stationary while latch retracts.
				if now.After(deliverTimer) {
					// Latch fully closed — credit the whole batch and decide what to do next.
					state.BallsCollected += state.BallsInHarvester
					state.BallsInHarvester = 0
					fmt.Printf("[DELIVER] Latch closed. Total delivered: %d/%d\n",
						state.BallsCollected, state.TotalBalls)
					state.DelivSubPhase = DelivSubTurn180
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
