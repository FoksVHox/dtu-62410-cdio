package vision

import (
	"fmt"
	"image"

	gocv "gocv.io/x/gocv"
)

// PerspectivePoint is a 2-D pixel coordinate used to specify a field corner.
type PerspectivePoint struct {
	X int
	Y int
}

// PerspectiveTransform holds the pre-computed homography matrix and the
// destination size used to warp raw camera frames into a rectified top-down
// view of the play field.
//
// Usage:
//
//	pt, err := NewPerspectiveTransform(cfg.Corners, cfg.OutputWidth, cfg.OutputHeight)
//	// ...in the render loop:
//	warped := gocv.NewMat()
//	defer warped.Close()
//	if err := pt.Warp(rawFrame, &warped); err != nil { ... }
//	// pass warped to Detector.Detect
type PerspectiveTransform struct {
	h   gocv.Mat // 3x3 homography matrix
	out image.Point
}

// NewPerspectiveTransform builds a homography from four source corners (in the
// raw camera frame) to the four corners of the output rectangle.
//
// corners must be exactly four points given in the order:
//
//	[0] top-left
//	[1] top-right
//	[2] bottom-right
//	[3] bottom-left
//
// outputWidth / outputHeight define the size of the rectified image.
func NewPerspectiveTransform(corners [4]PerspectivePoint, outputWidth, outputHeight int) (*PerspectiveTransform, error) {
	if outputWidth <= 0 || outputHeight <= 0 {
		return nil, fmt.Errorf("vision/perspective: output dimensions must be positive (got %dx%d)",
			outputWidth, outputHeight)
	}

	// Source: the four field corners as seen in the raw (possibly tilted) frame.
	src := gocv.NewPointVectorFromPoints([]image.Point{
		{X: corners[0].X, Y: corners[0].Y}, // top-left
		{X: corners[1].X, Y: corners[1].Y}, // top-right
		{X: corners[2].X, Y: corners[2].Y}, // bottom-right
		{X: corners[3].X, Y: corners[3].Y}, // bottom-left
	})
	defer src.Close()

	// Destination: the four corners of the rectified output rectangle.
	dst := gocv.NewPointVectorFromPoints([]image.Point{
		{X: 0, Y: 0},
		{X: outputWidth - 1, Y: 0},
		{X: outputWidth - 1, Y: outputHeight - 1},
		{X: 0, Y: outputHeight - 1},
	})
	defer dst.Close()

	h := gocv.GetPerspectiveTransform(src, dst)
	if h.Empty() {
		h.Close()
		return nil, fmt.Errorf("vision/perspective: failed to compute homography matrix")
	}

	return &PerspectiveTransform{
		h:   h,
		out: image.Pt(outputWidth, outputHeight),
	}, nil
}

// Warp applies the perspective correction to src and writes the rectified
// image into dst.  dst must be a valid (non-nil) *gocv.Mat; it will be
// (re-)allocated by WarpPerspective as needed.
func (p *PerspectiveTransform) Warp(src gocv.Mat, dst *gocv.Mat) error {
	if src.Empty() {
		return fmt.Errorf("vision/perspective: source frame is empty")
	}
	gocv.WarpPerspective(src, dst, p.h, p.out)
	if dst.Empty() {
		return fmt.Errorf("vision/perspective: warp produced an empty frame")
	}
	return nil
}

// Close releases the internal homography Mat.
func (p *PerspectiveTransform) Close() {
	p.h.Close()
}
