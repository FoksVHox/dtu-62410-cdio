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
	log.Debug("running in debug mode")
	motorCfg := config.Get().Mindstorm.Motors

	left, err := mindstorm.NewMotor(mindstorm.MotorConfig{
		Address:    motorCfg.Left.Address,
		DriverName: motorCfg.Left.DriverName,
		Inverted:   motorCfg.Left.Inverted,
	})
	if err != nil {
		log.WithError(err).Error("failed to initialize left motor")
		return
	}

	right, err := mindstorm.NewMotor(mindstorm.MotorConfig{
		Address:    motorCfg.Right.Address,
		DriverName: motorCfg.Right.DriverName,
		Inverted:   motorCfg.Right.Inverted,
	})
	if err != nil {
		log.WithError(err).Error("failed to initialize right motor")
		return
	}

	drive, err := mindstorm.NewBeltDrive(left, right)
	if err != nil {
		log.WithError(err).Error("failed to initialize belt drive")
		return
	}

	defer func() {
		if stopErr := drive.Stop(); stopErr != nil {
			log.WithError(stopErr).Error("failed to stop belt drive")
		}
	}()

	if err := drive.Drive(0.4); err != nil {
		log.WithError(err).Error("failed to start belt drive")
		return
	}

	log.Info("motors running for 30 seconds")
	time.Sleep(30 * time.Second)
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
