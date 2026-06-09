// Package vision provides GoCV-based computer vision utilities for the golfbot.
// It detects ping-pong balls (white/yellow spheres) from a top-down overhead
// camera feed and returns normalised position data that the navigation layer
// can consume.
//
// Coordinate system (top-down view of the play field):
//
//	X: -1 = left edge of frame,  +1 = right edge of frame
//	Y: -1 = near side of field (top of frame), +1 = far side (bottom of frame)
package vision

import (
	"fmt"
	"image"
	"image/color"
	"math"

	"bot/config"

	"github.com/apex/log"
	gocv "gocv.io/x/gocv"
)

// Ball holds the detected position and radius (in pixels) of a single ball.
type Ball struct {
	// Center of the detected circle in the camera frame.
	Center image.Point
	// Radius in pixels.
	Radius float64
}

// NormX returns the horizontal offset of the ball centre normalised to [-1, 1]
// where 0 is the image midpoint, -1 is the left edge and +1 the right edge.
func (b Ball) NormX(frameWidth int) float64 {
	if frameWidth <= 0 {
		return 0
	}
	return (float64(b.Center.X)/float64(frameWidth))*2.0 - 1.0
}

// NormY returns the depth offset of the ball centre normalised to [-1, 1]
// from the perspective of the top-down camera:
//
//	-1 = near side of the field (top of frame)
//	 0 = midpoint of the field
//	+1 = far side of the field (bottom of frame)
func (b Ball) NormY(frameHeight int) float64 {
	if frameHeight <= 0 {
		return 0
	}
	return (float64(b.Center.Y)/float64(frameHeight))*2.0 - 1.0
}

// EstimatedDistance returns a rough relative distance estimate from the apparent
// radius of the ball using: distance = K / radius.
func (b Ball) EstimatedDistance(k float64) float64 {
	if b.Radius <= 0 {
		return math.MaxFloat64
	}
	if k <= 0 {
		k = config.Get().Vision.DistanceK
	}
	return k / b.Radius
}

// Detector wraps the GoCV state required to detect balls in video frames.
type Detector struct {
	gray gocv.Mat
	blur gocv.Mat
	hsv  gocv.Mat
	mask gocv.Mat
}

// NewDetector allocates working Mats.  Call Close() when done.
func NewDetector() *Detector {
	return &Detector{
		gray: gocv.NewMat(),
		blur: gocv.NewMat(),
		hsv:  gocv.NewMat(),
		mask: gocv.NewMat(),
	}
}

// Close releases internal GoCV Mats.
func (d *Detector) Close() {
	d.gray.Close()
	d.blur.Close()
	d.hsv.Close()
	d.mask.Close()
}

// Detect runs ball detection on a BGR frame and returns all candidates sorted
// by descending radius (largest / closest ball first).
func (d *Detector) Detect(frame gocv.Mat) ([]Ball, error) {
	if frame.Empty() {
		return nil, fmt.Errorf("vision: empty frame")
	}

	cfg := config.Get().Vision

	// Convert to HSV for colour-based masking.
	gocv.CvtColor(frame, &d.hsv, gocv.ColorBGRToHSV)

	// Build a binary mask from the configured HSV range.
	low := gocv.NewScalar(
		float64(cfg.HSVLower.H),
		float64(cfg.HSVLower.S),
		float64(cfg.HSVLower.V),
		0,
	)
	high := gocv.NewScalar(
		float64(cfg.HSVUpper.H),
		float64(cfg.HSVUpper.S),
		float64(cfg.HSVUpper.V),
		0,
	)
	gocv.InRangeWithScalar(d.hsv, low, high, &d.mask)

	// Apply mask and blur to suppress noise.
	maskedBGR := gocv.NewMat()
	defer maskedBGR.Close()
	frame.CopyToWithMask(&maskedBGR, d.mask)

	blurKernel := cfg.BlurKernel
	if blurKernel <= 0 || blurKernel%2 == 0 {
		blurKernel = 9
	}
	gocv.GaussianBlur(maskedBGR, &d.blur, image.Pt(blurKernel, blurKernel), 0, 0, gocv.BorderDefault)
	gocv.CvtColor(d.blur, &d.gray, gocv.ColorBGRToGray)

	// Hough circle detection on the greyscale masked+blurred image.
	circles := gocv.NewMat()
	defer circles.Close()
	gocv.HoughCirclesWithParams(
		d.gray,
		&circles,
		gocv.HoughGradient,
		cfg.HoughDP,
		cfg.HoughMinDist,
		cfg.HoughParam1,
		cfg.HoughParam2,
		cfg.HoughMinRadius,
		cfg.HoughMaxRadius,
	)

	if circles.Empty() {
		return nil, nil
	}

	balls := make([]Ball, 0, circles.Cols())
	for i := 0; i < circles.Cols(); i++ {
		x := circles.GetFloatAt(0, i*3)
		y := circles.GetFloatAt(0, i*3+1)
		r := circles.GetFloatAt(0, i*3+2)
		balls = append(balls, Ball{
			Center: image.Pt(int(x), int(y)),
			Radius: float64(r),
		})
	}

	// Sort: largest radius (closest) first.
	sortBalls(balls)

	if cfg.DebugVision {
		d.drawDebug(frame, balls)
	}

	log.WithField("count", len(balls)).Debug("vision: detected balls")
	return balls, nil
}

func (d *Detector) drawDebug(frame gocv.Mat, balls []Ball) {
	for _, b := range balls {
		gocv.Circle(&frame, b.Center, int(b.Radius), color.RGBA{R: 0, G: 255, B: 0, A: 255}, 2)
		gocv.Circle(&frame, b.Center, 3, color.RGBA{R: 255, G: 0, B: 0, A: 255}, -1)
	}
}

func sortBalls(balls []Ball) {
	for i := 1; i < len(balls); i++ {
		for j := i; j > 0 && balls[j].Radius > balls[j-1].Radius; j-- {
			balls[j], balls[j-1] = balls[j-1], balls[j]
		}
	}
}
