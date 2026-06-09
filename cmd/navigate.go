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
LEGO robot toward each ball in turn, triggering the collector motor when close.

All parameters are read from config.yml (vision and navigation sections).`,
	RunE: runNavigate,
}

func init() {
	rootCommand.AddCommand(navigateCmd)
}

func runNavigate(_ *cobra.Command, _ []string) error {
	cfg := config.Get()

	// ── Camera & Detector ─────────────────────────────────────────────────────
	cam, err := vision.OpenCamera()
	if err != nil {
		return err
	}
	defer cam.Close()

	detector := vision.NewDetector()
	defer detector.Close()

	// ── Motors ────────────────────────────────────────────────────────────────
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

	// ── Navigator ─────────────────────────────────────────────────────────────
	nav := navigation.New(drive, collectMotor, cam, detector)

	// ── Run until SIGINT / SIGTERM ─────────────────────────────────────────────
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	log.Info("navigate: starting — press Ctrl-C to stop")
	if err := nav.Run(ctx); err != nil && err != context.Canceled {
		return err
	}
	return nil
}
