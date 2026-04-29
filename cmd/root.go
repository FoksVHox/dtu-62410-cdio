package cmd

import (
	"bot/config"
	"bot/mindstorm"
	log2 "log"
	"path/filepath"
	"time"

	"github.com/NYTimes/logrotate"
	"github.com/apex/log"
	"github.com/apex/log/handlers/cli"
	"github.com/apex/log/handlers/multi"
	"github.com/spf13/cobra"
	"gocv.io/x/gocv"
)

var (
	configPath = config.DefaultLocation
	debug      = false
)

var rootCommand = &cobra.Command{
	Use:   "bot",
	Short: "Runs the bot.",
	PreRun: func(cmd *cobra.Command, args []string) {
		initConfig()
		initLogging()
	},
	Run: rootCmdRun,
}

func Execute() {
	if err := rootCommand.Execute(); err != nil {
		log2.Fatalf("failed to execute command: %s", err)
	}
}

func init() {
	rootCommand.PersistentFlags().StringVar(&configPath, "config", config.DefaultLocation, "set the location for the configuration file")
	rootCommand.PersistentFlags().BoolVar(&debug, "debug", false, "pass in order to run bot in debug mode")
}

func rootCmdRun(cmd *cobra.Command, _ []string) {
	log.WithField("command", cmd.Name()).Debug("running in debug mode")
	motorCfg := config.Get().Mindstorm.Motors
	log.WithFields(log.Fields{
		"left_address":      motorCfg.Left.Address,
		"left_driver_name":  motorCfg.Left.DriverName,
		"left_inverted":     motorCfg.Left.Inverted,
		"right_address":     motorCfg.Right.Address,
		"right_driver_name": motorCfg.Right.DriverName,
		"right_inverted":    motorCfg.Right.Inverted,
	}).Debug("loaded motor configuration")

	left, err := mindstorm.NewMotor(mindstorm.MotorConfig{
		Address:    motorCfg.Left.Address,
		DriverName: motorCfg.Left.DriverName,
		Inverted:   motorCfg.Left.Inverted,
	})
	if err != nil {
		log.WithError(err).Error("failed to initialize left motor")
		return
	}
	log.Debug("left motor initialized")

	right, err := mindstorm.NewMotor(mindstorm.MotorConfig{
		Address:    motorCfg.Right.Address,
		DriverName: motorCfg.Right.DriverName,
		Inverted:   motorCfg.Right.Inverted,
	})
	if err != nil {
		log.WithError(err).Error("failed to initialize right motor")
		return
	}
	log.Debug("right motor initialized")

	drive, err := mindstorm.NewBeltDrive(left, right)
	if err != nil {
		log.WithError(err).Error("failed to initialize belt drive")
		return
	}
	log.Debug("belt drive initialized")

	defer func() {
		log.Debug("stopping belt drive")
		if stopErr := drive.Stop(); stopErr != nil {
			log.WithError(stopErr).Error("failed to stop belt drive")
			return
		}
		log.Debug("belt drive stopped")
	}()

	if err := drive.Drive(0.4); err != nil {
		log.WithError(err).Error("failed to start belt drive")
		return
	}
	log.WithField("throttle", 0.4).Debug("belt drive started")

	log.Info("motors running for 30 seconds")
	time.Sleep(30 * time.Second)

	webcam, _ := gocv.VideoCaptureDevice(0)
	window := gocv.NewWindow("Hello")
	img := gocv.NewMat()

	for {
		webcam.Read(&img)
		window.IMShow(img)
		window.WaitKey(1)
	}
}

// Reads the configuration from the disk and then sets up the global singleton
// with all the configuration values.
func initConfig() {
	if !filepath.IsAbs(configPath) {
		d, err := filepath.Abs(configPath)
		if err != nil {
			log2.Fatalf("cmd/root: failed to get path to config file: %s", err)
		}
		configPath = d
	}

	err := config.FromFile(configPath)
	if err != nil {
		log2.Fatalf("cmd/root: error while reading configuration file: %s", err)
	}
	if debug && !config.Get().Debug {
		config.SetDebugViaFlag(debug)
	}
}

// Configures the global logger for Zap so that we can call it from any location
// in the code without having to pass around a logger instance.
func initLogging() {
	dir := config.Get().System.LogDirectory
	p := filepath.Join(dir, "/bot.log")
	w, err := logrotate.NewFile(p)
	if err != nil {
		log2.Fatalf("cmd/root: failed to create bot log: %s", err)
	}
	log.SetLevel(log.InfoLevel)
	if config.Get().Debug {
		log.SetLevel(log.DebugLevel)
	}
	log.SetHandler(multi.New(cli.Default, cli.New(w.File)))
	log.WithField("path", p).Info("writing log files to disk")
}
