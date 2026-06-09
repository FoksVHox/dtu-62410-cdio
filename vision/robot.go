package vision

import (
	"fmt"
	"image"
	"math"

	"bot/config"

	gocv "gocv.io/x/gocv"
)

type markerDetection struct {
	found  bool
	center image.Point
	area   float64
}

type RobotDetector struct {
	hsv         gocv.Mat
	bodyMask    gocv.Mat
	frontMask   gocv.Mat
	bodyKernel  gocv.Mat
	frontKernel gocv.Mat
}

func NewRobotDetector() *RobotDetector {
	return &RobotDetector{
		hsv:         gocv.NewMat(),
		bodyMask:    gocv.NewMat(),
		frontMask:   gocv.NewMat(),
		bodyKernel:  gocv.GetStructuringElement(gocv.MorphEllipse, image.Pt(5, 5)),
		frontKernel: gocv.GetStructuringElement(gocv.MorphEllipse, image.Pt(5, 5)),
	}
}

func (r *RobotDetector) Close() {
	r.hsv.Close()
	r.bodyMask.Close()
	r.frontMask.Close()
	r.bodyKernel.Close()
	r.frontKernel.Close()
}

func (r *RobotDetector) Detect(frame gocv.Mat) (RobotPose, error) {
	if frame.Empty() {
		return RobotPose{}, fmt.Errorf("vision: empty frame for robot detection")
	}

	cfg := config.Get().Vision
	gocv.CvtColor(frame, &r.hsv, gocv.ColorBGRToHSV)

	body := buildMask(r.hsv, &r.bodyMask, cfg.RobotBodyLower, cfg.RobotBodyUpper, r.bodyKernel)
	front := buildMask(r.hsv, &r.frontMask, cfg.RobotFrontLower, cfg.RobotFrontUpper, r.frontKernel)

	if !body.found || !front.found {
		return RobotPose{Detected: false}, nil
	}

	heading := math.Atan2(float64(front.center.Y-body.center.Y), float64(front.center.X-body.center.X))

	return RobotPose{
		Detected: true,
		X:        normCoord(body.center.X, frame.Cols()),
		Y:        normCoord(body.center.Y, frame.Rows()),
		Heading:  heading,
		BodyX:    body.center.X,
		BodyY:    body.center.Y,
		FrontX:   front.center.X,
		FrontY:   front.center.Y,
	}, nil
}

func buildMask(hsv gocv.Mat, dst *gocv.Mat, low, high config.HSVBound, kernel gocv.Mat) markerDetection {
	lower := gocv.NewScalar(float64(low.H), float64(low.S), float64(low.V), 0)
	upper := gocv.NewScalar(float64(high.H), float64(high.S), float64(high.V), 0)

	gocv.InRangeWithScalar(hsv, lower, upper, dst)
	gocv.MorphologyEx(*dst, dst, gocv.MorphOpen, kernel)
	gocv.MorphologyEx(*dst, dst, gocv.MorphClose, kernel)

	contours := gocv.FindContours(*dst, gocv.RetrievalExternal, gocv.ChainApproxSimple)
	bestArea := 0.0
	bestCenter := image.Point{}

	for i := range contours {
		area := gocv.ContourArea(contours[i])
		if area <= bestArea {
			continue
		}
		m := gocv.Moments(contours[i], false)
		if m["m00"] == 0 {
			continue
		}
		cx := int(m["m10"] / m["m00"])
		cy := int(m["m01"] / m["m00"])
		bestArea = area
		bestCenter = image.Pt(cx, cy)
	}

	if bestArea <= 0 {
		return markerDetection{found: false}
	}
	return markerDetection{found: true, center: bestCenter, area: bestArea}
}

func normCoord(v, size int) float64 {
	if size <= 0 {
		return 0
	}
	return (float64(v)/float64(size))*2.0 - 1.0
}
