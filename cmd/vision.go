package cmd

import (
	"context"
	"os"
	"os/signal"
	"syscall"

	"bot/vision"

	"github.com/apex/log"
	"github.com/spf13/cobra"
)

var visionCmd = &cobra.Command{
	Use:   "vision",
	Short: "Start the PC-side live vision server",
	Long: `Runs the continuous top-down camera processing loop on the PC.

It:
- opens the camera
- applies perspective correction when enabled
- detects balls
- detects robot pose from two HSV markers
- serves a live MJPEG stream on /stream
- serves JSON world state on /state`,
	RunE: runVision,
}

func init() {
	rootCommand.AddCommand(visionCmd)
}

func runVision(_ *cobra.Command, _ []string) error {
	svc, err := vision.NewService()
	if err != nil {
		return err
	}
	defer svc.Close()

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	log.Info("vision: starting — press Ctrl-C to stop")
	return svc.Run(ctx)
}
