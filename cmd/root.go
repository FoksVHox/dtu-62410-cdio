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
		"head_address":      motorCfg.Head.Address,
		"head_driver_name":  motorCfg.Head.DriverName,
		"head_inverted":     motorCfg.Head.Inverted,
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

	head, err := mindstorm.NewMotor(mindstorm.MotorConfig{
		Address:    motorCfg.Head.Address,
		DriverName: motorCfg.Head.DriverName,
		Inverted:   motorCfg.Head.Inverted,
	})
	if err != nil {
		log.WithError(err).Error("failed to initialize head motor")
		return
	}
	log.Debug("head motor initialized")

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
		}
		log.Debug("belt drive stopped")

		log.Debug("stopping head motor")
		if stopErr := head.Stop(config.Get().Mindstorm.EV3.DefaultStopAction); stopErr != nil {
			log.WithError(stopErr).Error("failed to stop head motor")
		}
		log.Debug("head motor stopped")
	}()

	// Test belt drive
	log.Info("starting belt drive test")
	if err := drive.Drive(0.4); err != nil {
		log.WithError(err).Error("failed to start belt drive")
		return
	}
	log.WithField("throttle", 0.4).Info("belt drive started")

	testDuration := motorCfg.MotorTestTime
	log.WithField("duration_seconds", testDuration).Info("belt drive will run for configured duration")
	time.Sleep(time.Duration(testDuration) * time.Second)
	log.Info("belt drive test duration complete")

	// Test head motor
	log.Info("starting head motor test")
	headSpeed := int(float64(head.MaxSpeedTPS()) * 0.5) // 50% speed
	if err := head.RunTimed(headSpeed, int(testDuration)); err != nil {
		log.WithError(err).Error("failed to start head motor")
		return
	}
	log.WithFields(log.Fields{
		"speed_tps":        headSpeed,
		"duration_seconds": testDuration,
	}).Info("head motor started with timed run")

	time.Sleep(time.Duration(testDuration) * time.Second)
	log.Info("head motor test duration complete")
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
