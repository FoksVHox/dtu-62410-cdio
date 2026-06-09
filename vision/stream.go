package vision

import (
	"context"
	"encoding/json"
	"fmt"
	"image"
	"image/color"
	"net/http"
	"sync"
	"time"

	"bot/config"

	"github.com/apex/log"
	gocv "gocv.io/x/gocv"
)

type Service struct {
	cam         *Camera
	detector    *Detector
	robot       *RobotDetector
	perspective *PerspectiveTransform

	mu        sync.RWMutex
	lastJPEG  []byte
	lastState WorldState

	server *http.Server
}

func NewService() (*Service, error) {
	cfg := config.Get().Vision

	cam, err := OpenCamera()
	if err != nil {
		return nil, err
	}

	detector := NewDetector()
	robot := NewRobotDetector()

	var perspective *PerspectiveTransform
	if cfg.Perspective.Enabled {
		corners := [4]PerspectivePoint{
			{X: cfg.Perspective.TopLeft.X, Y: cfg.Perspective.TopLeft.Y},
			{X: cfg.Perspective.TopRight.X, Y: cfg.Perspective.TopRight.Y},
			{X: cfg.Perspective.BottomRight.X, Y: cfg.Perspective.BottomRight.Y},
			{X: cfg.Perspective.BottomLeft.X, Y: cfg.Perspective.BottomLeft.Y},
		}
		perspective, err = NewPerspectiveTransform(corners, cfg.Perspective.OutputWidth, cfg.Perspective.OutputHeight)
		if err != nil {
			_ = cam.Close()
			detector.Close()
			robot.Close()
			return nil, err
		}
	}

	s := &Service{
		cam:         cam,
		detector:    detector,
		robot:       robot,
		perspective: perspective,
	}

	mux := http.NewServeMux()
	mux.HandleFunc("/state", s.handleState)
	mux.HandleFunc("/stream", s.handleStream)
	mux.HandleFunc("/", s.handleIndex)

	s.server = &http.Server{
		Addr:    fmt.Sprintf("%s:%d", cfg.StreamBind, cfg.StreamPort),
		Handler: mux,
	}

	return s, nil
}

func (s *Service) Close() {
	if s.server != nil {
		_ = s.server.Close()
	}
	if s.perspective != nil {
		s.perspective.Close()
	}
	if s.robot != nil {
		s.robot.Close()
	}
	if s.detector != nil {
		s.detector.Close()
	}
	if s.cam != nil {
		_ = s.cam.Close()
	}
}

func (s *Service) Run(ctx context.Context) error {
	errCh := make(chan error, 1)

	go func() {
		log.WithField("addr", s.server.Addr).Info("vision: HTTP server listening")
		if err := s.server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			errCh <- err
		}
	}()

	go func() {
		errCh <- s.loop(ctx)
	}()

	select {
	case <-ctx.Done():
		shutdownCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()
		_ = s.server.Shutdown(shutdownCtx)
		return ctx.Err()
	case err := <-errCh:
		shutdownCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()
		_ = s.server.Shutdown(shutdownCtx)
		return err
	}
}

func (s *Service) loop(ctx context.Context) error {
	raw := gocv.NewMat()
	defer raw.Close()

	warped := gocv.NewMat()
	defer warped.Close()

	annotated := gocv.NewMat()
	defer annotated.Close()

	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
		}

		if err := s.cam.Read(&raw); err != nil {
			return err
		}

		work := raw
		if s.perspective != nil {
			if err := s.perspective.Warp(raw, &warped); err == nil {
				work = warped
			}
		}

		balls, err := s.detector.Detect(work)
		if err != nil {
			return err
		}

		robotPose, err := s.robot.Detect(work)
		if err != nil {
			return err
		}

		work.CopyTo(&annotated)
		drawOverlay(&annotated, balls, robotPose)

		state := WorldState{
			Timestamp:   time.Now(),
			FrameWidth:  annotated.Cols(),
			FrameHeight: annotated.Rows(),
			Robot:       robotPose,
			Balls:       make([]BallState, 0, len(balls)),
		}
		for _, b := range balls {
			state.Balls = append(state.Balls, BallState{
				X:      b.Center.X,
				Y:      b.Center.Y,
				NormX:  b.NormX(annotated.Cols()),
				NormY:  b.NormY(annotated.Rows()),
				Radius: b.Radius,
			})
		}

		buf, err := gocv.IMEncode(".jpg", annotated)
		if err != nil {
			return err
		}
		jpeg := append([]byte(nil), buf.GetBytes()...)
		buf.Close()

		s.mu.Lock()
		s.lastJPEG = jpeg
		s.lastState = state
		s.mu.Unlock()

		time.Sleep(33 * time.Millisecond)
	}
}

func drawOverlay(frame *gocv.Mat, balls []Ball, robot RobotPose) {
	for _, b := range balls {
		gocv.Circle(frame, b.Center, int(b.Radius), color.RGBA{G: 255, A: 255}, 2)
		gocv.Circle(frame, b.Center, 3, color.RGBA{R: 255, A: 255}, -1)
	}

	if robot.Detected {
		body := imagePt(robot.BodyX, robot.BodyY)
		front := imagePt(robot.FrontX, robot.FrontY)
		gocv.Circle(frame, body, 8, color.RGBA{B: 255, A: 255}, -1)
		gocv.Circle(frame, front, 8, color.RGBA{R: 255, A: 255}, -1)
		gocv.Line(frame, body, front, color.RGBA{R: 255, G: 255, A: 255}, 2)
	}
}

func imagePt(x, y int) image.Point {
	return image.Point{X: x, Y: y}
}

func (s *Service) handleState(w http.ResponseWriter, _ *http.Request) {
	s.mu.RLock()
	state := s.lastState
	s.mu.RUnlock()

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(state)
}

func (s *Service) handleStream(w http.ResponseWriter, _ *http.Request) {
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "close")
	w.Header().Set("Content-Type", "multipart/x-mixed-replace; boundary=frame")

	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for range ticker.C {
		s.mu.RLock()
		jpeg := append([]byte(nil), s.lastJPEG...)
		s.mu.RUnlock()

		if len(jpeg) == 0 {
			continue
		}

		_, _ = fmt.Fprintf(w, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", len(jpeg))
		_, _ = w.Write(jpeg)
		_, _ = w.Write([]byte("\r\n"))

		if f, ok := w.(http.Flusher); ok {
			f.Flush()
		}
	}
}

func (s *Service) handleIndex(w http.ResponseWriter, _ *http.Request) {
	_, _ = w.Write([]byte(`<html><body style="background:#111;color:#eee;font-family:sans-serif">
<h1>Golfbot vision</h1>
<p><a href="/stream">MJPEG stream</a></p>
<p><a href="/state">JSON state</a></p>
<img src="/stream" style="max-width:100%;border:1px solid #444" />
</body></html>`))
}
