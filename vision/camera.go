package vision

import (
	"fmt"
	"sync"

	"bot/config"

	"github.com/apex/log"
	gocv "gocv.io/x/gocv"
)

// Camera wraps gocv.VideoCapture and provides thread-safe frame grabbing.
//
// The camera is mounted in a top-down (overhead) orientation looking straight
// down at the play field.  Frames are used as-is — no horizontal mirroring is
// applied.  Coordinate conventions (see detector.go):
//
//	X: left edge = -1, right edge = +1
//	Y: near side of field (top of frame) = -1, far side (bottom) = +1
type Camera struct {
	mu  sync.Mutex
	cap *gocv.VideoCapture
	dev int
}

// OpenCamera opens the video device described by the global config.
func OpenCamera() (*Camera, error) {
	cfg := config.Get().Vision

	cap, err := gocv.OpenVideoCapture(cfg.CameraDevice)
	if err != nil {
		return nil, fmt.Errorf("vision: open camera %d: %w", cfg.CameraDevice, err)
	}

	if cfg.CameraWidth > 0 {
		cap.Set(gocv.VideoCaptureFrameWidth, float64(cfg.CameraWidth))
	}
	if cfg.CameraHeight > 0 {
		cap.Set(gocv.VideoCaptureFrameHeight, float64(cfg.CameraHeight))
	}
	if cfg.CameraFPS > 0 {
		cap.Set(gocv.VideoCaptureFPS, cfg.CameraFPS)
	}

	log.WithFields(log.Fields{
		"device":      cfg.CameraDevice,
		"width":       cfg.CameraWidth,
		"height":      cfg.CameraHeight,
		"fps":         cfg.CameraFPS,
		"orientation": "top-down",
	}).Info("vision: camera opened")

	return &Camera{cap: cap, dev: cfg.CameraDevice}, nil
}

// Read grabs the next frame into dst.  Returns an error if the frame is empty.
// The frame is returned in its natural top-down orientation — no mirroring is
// performed.
func (c *Camera) Read(dst *gocv.Mat) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if ok := c.cap.Read(dst); !ok {
		return fmt.Errorf("vision: failed to read frame from camera %d", c.dev)
	}
	if dst.Empty() {
		return fmt.Errorf("vision: empty frame from camera %d", c.dev)
	}
	return nil
}

// Width returns the actual frame width reported by the driver.
func (c *Camera) Width() int {
	return int(c.cap.Get(gocv.VideoCaptureFrameWidth))
}

// Height returns the actual frame height reported by the driver.
func (c *Camera) Height() int {
	return int(c.cap.Get(gocv.VideoCaptureFrameHeight))
}

// Close releases the underlying VideoCapture.
func (c *Camera) Close() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.cap.Close()
}
