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

// minCircularity is the minimum circularity score (4π·area/perimeter²) a
// contour must achieve to be considered a ball candidate.  A perfect circle
// scores 1.0; real golf balls typically score ≥ 0.70.  Irregular light
// reflections and hot-spots score lower and are rejected by this filter.
// Decrease toward 0.60 if real balls are missed; increase toward 0.80 to
// reject more ghost blobs.
const minCircularity = 0.70

// ── Goal-delivery tuning ──────────────────────────────────────────────────────

// deliverTurn180Tol is the heading error (degrees) at which the 180° turn is
// considered complete and we switch to reversing.
const deliverTurn180Tol = 8.0

// deliverBackupSpeed is the reverse throttle magnitude used when backing into
// the goal (positive value; sign is applied inside ForceReverse).
const deliverBackupSpeed = 0.40

// deliverGoalArrivalPx is the pixel distance from the goal centre at which we
// consider the robot close enough to open the latch.
const deliverGoalArrivalPx = 70.0

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

					// ── Round-shape filter ────────────────────────────────────
					// Circularity = 4π·area / perimeter².  A perfect circle
					// scores 1.0; real golf balls ≥ ~0.70.  Light reflections
					// and hot-spots are typically elongated or jagged and score
					// lower, so they are rejected here before the sightings
					// tracker ever considers them.
					perimeter := gocv.ArcLength(contour, true)
					if perimeter <= 0 {
						continue
					}
					circularity := 4.0 * math.Pi * float64(area) / (perimeter * perimeter)
					if circularity < minCircularity {
						continue
					}
					// ─────────────────────────────────────────────────────────

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

			// ── Step 1: Spin 180° so the robot's back faces the goal ─────────────
			case DelivSubTurn180:
				if !goal.Detected {
					robotLink.Stop()
					gocv.PutText(&img, "DELIVER TURN180: waiting for goal marker",
						image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
					break
				}

				dx := float64(goal.Center.X - robot.Center.X)
				dy := float64(goal.Center.Y - robot.Center.Y)
				bearingToGoal := math.Atan2(dy, dx) * 180.0 / math.Pi
				if bearingToGoal < 0 {
					bearingToGoal += 360
				}
				targetHeading := math.Mod(bearingToGoal+180, 360)
				headingErr := normaliseAngle(targetHeading - robot.Angle)
				absErr := math.Abs(headingErr)

				gocv.PutText(&img,
					fmt.Sprintf("DELIVER: TURN180 err=%.0f°", headingErr),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
				fmt.Printf("[DELIVER] TURN180 | robotAngle=%.1f° targetHeading=%.1f° err=%.2f°\n",
					robot.Angle, targetHeading, headingErr)

				if absErr <= deliverTurn180Tol {
					fmt.Println("[DELIVER] TURN180 complete — switching to BACK_UP")
					robotLink.Stop()
					state.DelivSubPhase = DelivSubBackUp
					break
				}

				turnSign := math.Copysign(1, headingErr)
				turnMag := math.Min(absErr/15.0, 1.0) * 0.4
				robotLink.Send(DriveCommand{Throttle: 0, Turn: turnSign * turnMag})

			// ── Step 2: Reverse straight into the goal ───────────────────────────
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
					fmt.Sprintf("DELIVER: BACK_UP dist=%.0fpx", dist),
					image.Pt(20, 100), gocv.FontHersheySimplex, 0.6, magentaColor, 2)
				gocv.Line(&img, robot.Center, goal.Center, magentaColor, 1)
				fmt.Printf("[DELIVER] BACK_UP | dist=%.1fpx goal=(%d,%d) robot=(%d,%d)\n",
					dist, goal.Center.X, goal.Center.Y, robot.Center.X, robot.Center.Y)

				if dist <= deliverGoalArrivalPx {
					fmt.Println("[DELIVER] BACK_UP complete — robot in goal, opening latch")
					robotLink.Stop()
					state.DelivSubPhase = DelivSubOpenLatch
					break
				}

				bearingToGoal := math.Atan2(dy, dx) * 180.0 / math.Pi
				if bearingToGoal < 0 {
					bearingToGoal += 360
				}
				backFacing := math.Mod(robot.Angle+180, 360)
				steerErr := normaliseAngle(bearingToGoal - backFacing)
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
