package cmd

import (
	"context"
	"os"
	"os/signal"
	"syscall"

	"bot/config"
	"bot/mindstorm"
	"bot/navigation"
	"bot/vision"

	"github.com/apex/log"
	"github.com/spf13/cobra"
)

var navigateCmd = &cobra.Command{
	Use:   "navigate",
	Short: "Start the GoCV-based ball navigation loop",
	Long: `Opens the camera, detects ping-pong balls via Hough circles and drives the
LEGO robot toward each ball in turn, triggering the collector motor when close.`,
	RunE: runNavigate,
}

func init() {
	rootCmd.AddCommand(navigateCmd)

	navigateCmd.Flags().Int("camera", 0, "Camera device index (default 0)")
	navigateCmd.Flags().Int("width", 640, "Capture width in pixels")
	navigateCmd.Flags().Int("height", 480, "Capture height in pixels")
	navigateCmd.Flags().Bool("debug-vision", false, "Show OpenCV debug overlay in a window")
	navigateCmd.Flags().Float64("drive-speed", 0.35, "Base forward throttle [0,1]")
	navigateCmd.Flags().Float64("turn-speed", 0.25, "Search rotation speed [0,1]")
	navigateCmd.Flags().Float64("steering-gain", 0.6, "Proportional steering gain")
	navigateCmd.Flags().Float64("collect-dist", 1.5, "Estimated distance threshold to trigger collection")
}

func runNavigate(cmd *cobra.Command, _ []string) error {
	cfg := config.Get()

	// ── Camera ───────────────────────────────────────────────────────────────
	camIdx, _ := cmd.Flags().GetInt("camera")
	camW, _ := cmd.Flags().GetInt("width")
	camH, _ := cmd.Flags().GetInt("height")

	camCfg := vision.CameraConfig{
		DeviceIndex: camIdx,
		Width:       camW,
		Height:      camH,
		FPS:         30,
	}
	cam, err := vision.OpenCamera(camCfg)
	if err != nil {
		return err
	}
	defer cam.Close()

	// ── Detector ─────────────────────────────────────────────────────────────
	dbgVision, _ := cmd.Flags().GetBool("debug-vision")
	detCfg := vision.DefaultDetectorConfig()
	detCfg.Debug = dbgVision
	detector := vision.NewDetector(detCfg)
	defer detector.Close()

	// ── Motors ───────────────────────────────────────────────────────────────
	leftMotor, err := mindstorm.NewMotor(mindstorm.MotorConfig{
		Address:  cfg.Mindstorm.Motors.Left.Address,
		Inverted: cfg.Mindstorm.Motors.Left.Inverted,
	})
	if err != nil {
		return err
	}

	rightMotor, err := mindstorm.NewMotor(mindstorm.MotorConfig{
		Address:  cfg.Mindstorm.Motors.Right.Address,
		Inverted: cfg.Mindstorm.Motors.Right.Inverted,
	})
	if err != nil {
		return err
	}

	drive, err := mindstorm.NewBeltDrive(leftMotor, rightMotor)
	if err != nil {
		return err
	}

	// Collector motor is optional — nil is handled gracefully by the navigator.
	var collectMotor *mindstorm.Motor
	if cfg.Mindstorm.Motors.Back.Address != "" {
		collectMotor, err = mindstorm.NewMotor(mindstorm.MotorConfig{
			Address:  cfg.Mindstorm.Motors.Back.Address,
			Inverted: cfg.Mindstorm.Motors.Back.Inverted,
		})
		if err != nil {
			log.WithError(err).Warn("navigate: collector motor unavailable")
			collectMotor = nil
		}
	}

	// ── Navigation config ─────────────────────────────────────────────────────
	navCfg := navigation.DefaultConfig()
	navCfg.DriveSpeed, _ = cmd.Flags().GetFloat64("drive-speed")
	navCfg.TurnSpeed, _ = cmd.Flags().GetFloat64("turn-speed")
	navCfg.SteeringGain, _ = cmd.Flags().GetFloat64("steering-gain")
	navCfg.CollectDistanceThreshold, _ = cmd.Flags().GetFloat64("collect-dist")

	nav := navigation.New(navCfg, drive, collectMotor, cam, detector)

	// ── Run until SIGINT/SIGTERM ──────────────────────────────────────────────
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	log.Info("navigate: starting — press Ctrl-C to stop")
	if err := nav.Run(ctx); err != nil && err != context.Canceled {
		return err
	}
	return nil
}
