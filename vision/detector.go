// Package vision provides GoCV-based computer vision utilities for the golfbot.
// It detects ping-pong balls (white/yellow spheres) from a camera feed and
// returns normalised position data that the navigation layer can consume.
package vision

import (
	"fmt"
	"image"
	"image/color"
	"math"

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

// NormY returns the vertical offset normalised to [-1, 1] where 0 is the image
// midpoint, -1 is the top edge and +1 the bottom edge.
func (b Ball) NormY(frameHeight int) float64 {
	if frameHeight <= 0 {
		return 0
	}
	return (float64(b.Center.Y)/float64(frameHeight))*2.0 - 1.0
}

// EstimatedDistance returns a rough relative distance estimate from the apparent
// radius of the ball.  The constant K is tuned empirically; increase it if the
// robot stops too far from the ball.
func (b Ball) EstimatedDistance(k float64) float64 {
	if b.Radius <= 0 {
		return math.MaxFloat64
	}
	if k <= 0 {
		k = DetectorDefaultDistanceK
	}
	return k / b.Radius
}

// DetectorDefaultDistanceK is the empirical constant used to estimate ball
// distance from its apparent pixel radius.  Tune this on the real robot.
const DetectorDefaultDistanceK = 200.0

// DetectorConfig holds tunable parameters for the ball detector.
type DetectorConfig struct {
	// HSV lower bound for the ball colour (H 0-179, S 0-255, V 0-255).
	// Default targets white / light-yellow ping-pong balls.
	HSVLower color.RGBA
	// HSV upper bound.
	HSVUpper color.RGBA

	// Hough circle detection parameters.
	DP              float64 // inverse ratio of accumulator resolution (1–2)
	MinDist         float64 // minimum distance between detected centres (px)
	Param1          float64 // Canny high threshold
	Param2          float64 // accumulator threshold (lower → more false positives)
	MinRadius       int     // minimum circle radius to accept
	MaxRadius       int     // maximum circle radius to accept  (0 = no limit)

	// GaussianBlur kernel size (must be odd).
	BlurKernel int

	// DistanceK for EstimatedDistance.
	DistanceK float64

	// Debug draws detection overlay onto a window when true.
	Debug bool
}

// DefaultDetectorConfig returns a sensible starting configuration for detecting
// white / light-yellow ping-pong balls under indoor lighting.
func DefaultDetectorConfig() DetectorConfig {
	return DetectorConfig{
		// White-ish: low saturation, high value
		HSVLower: color.RGBA{R: 0, G: 0, B: 180, A: 255},  // H, S, V
		HSVUpper: color.RGBA{R: 179, G: 60, B: 255, A: 255},
		DP:        1.2,
		MinDist:   30,
		Param1:    100,
		Param2:    20,
		MinRadius: 8,
		MaxRadius: 80,
		BlurKernel: 9,
		DistanceK: DetectorDefaultDistanceK,
	}
}

// Detector wraps the GoCV state required to detect balls in video frames.
type Detector struct {
	cfg  DetectorConfig
	gray gocv.Mat
	blur gocv.Mat
	hsv  gocv.Mat
	mask gocv.Mat
}

// NewDetector allocates working Mats.  Call Close() when done.
func NewDetector(cfg DetectorConfig) *Detector {
	return &Detector{
		cfg:  cfg,
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

	// Convert to HSV for colour-based masking.
	gocv.CvtColor(frame, &d.hsv, gocv.ColorBGRToHSV)

	// Build a binary mask from the configured HSV range.
	low := gocv.NewScalar(
		float64(d.cfg.HSVLower.R),
		float64(d.cfg.HSVLower.G),
		float64(d.cfg.HSVLower.B),
		0,
	)
	high := gocv.NewScalar(
		float64(d.cfg.HSVUpper.R),
		float64(d.cfg.HSVUpper.G),
		float64(d.cfg.HSVUpper.B),
		0,
	)
	gocv.InRangeWithScalar(d.hsv, low, high, &d.mask)

	// Apply mask and blur to suppress noise.
	maskedBGR := gocv.NewMat()
	defer maskedBGR.Close()
	frame.CopyToWithMask(&maskedBGR, d.mask)

	blurKernel := d.cfg.BlurKernel
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
		d.cfg.DP,
		d.cfg.MinDist,
		d.cfg.Param1,
		d.cfg.Param2,
		d.cfg.MinRadius,
		d.cfg.MaxRadius,
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

	if d.cfg.Debug {
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
