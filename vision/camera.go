package vision

import (
	"fmt"
	"sync"

	"github.com/apex/log"
	gocv "gocv.io/x/gocv"
)

// CameraConfig holds configuration for opening a video capture source.
type CameraConfig struct {
	// DeviceIndex is the index passed to OpenVideoCapture (0 = /dev/video0).
	DeviceIndex int
	// Width / Height for the capture resolution (0 = driver default).
	Width  int
	Height int
	// FPS hint for the driver (0 = driver default).
	FPS float64
}

// DefaultCameraConfig returns a sensible default for the EV3 / Raspberry Pi camera.
func DefaultCameraConfig() CameraConfig {
	return CameraConfig{
		DeviceIndex: 0,
		Width:       640,
		Height:      480,
		FPS:         30,
	}
}

// Camera wraps gocv.VideoCapture and provides thread-safe frame grabbing.
type Camera struct {
	mu  sync.Mutex
	cap *gocv.VideoCapture
	cfg CameraConfig
}

// OpenCamera opens the video device described by cfg.
func OpenCamera(cfg CameraConfig) (*Camera, error) {
	cap, err := gocv.OpenVideoCapture(cfg.DeviceIndex)
	if err != nil {
		return nil, fmt.Errorf("vision: open camera %d: %w", cfg.DeviceIndex, err)
	}

	if cfg.Width > 0 {
		cap.Set(gocv.VideoCaptureFrameWidth, float64(cfg.Width))
	}
	if cfg.Height > 0 {
		cap.Set(gocv.VideoCaptureFrameHeight, float64(cfg.Height))
	}
	if cfg.FPS > 0 {
		cap.Set(gocv.VideoCaptureFPS, cfg.FPS)
	}

	log.WithFields(log.Fields{
		"device": cfg.DeviceIndex,
		"width":  cfg.Width,
		"height": cfg.Height,
		"fps":    cfg.FPS,
	}).Info("vision: camera opened")

	return &Camera{cap: cap, cfg: cfg}, nil
}

// Read grabs the next frame into dst.  Returns an error if the frame is empty.
func (c *Camera) Read(dst *gocv.Mat) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if ok := c.cap.Read(dst); !ok {
		return fmt.Errorf("vision: failed to read frame from camera %d", c.cfg.DeviceIndex)
	}
	if dst.Empty() {
		return fmt.Errorf("vision: empty frame from camera %d", c.cfg.DeviceIndex)
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
