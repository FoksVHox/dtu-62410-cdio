package cmd

import (
	"bot/config"
	"bot/mindstorm"
	log2 "log"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
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
		"left_speed":        motorCfg.Left.Speed,
		"right_address":     motorCfg.Right.Address,
		"right_driver_name": motorCfg.Right.DriverName,
		"right_inverted":    motorCfg.Right.Inverted,
		"right_speed":       motorCfg.Right.Speed,
		"head_address":      motorCfg.Head.Address,
		"head_driver_name":  motorCfg.Head.DriverName,
		"head_inverted":     motorCfg.Head.Inverted,
		"head_speed":        motorCfg.Head.Speed,
		"motor_test_time":   motorCfg.MotorTestTime,
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

	back, err := mindstorm.NewMotor(mindstorm.MotorConfig{
		Address:    motorCfg.Back.Address,
		DriverName: motorCfg.Back.DriverName,
		Inverted:   motorCfg.Back.Inverted,
	})
	if err != nil {
		log.WithError(err).Error("failed to initialize back motor")
		return
	}
	log.Debug("back motor initialized")

	drive, err := mindstorm.NewBeltDrive(left, right)
	if err != nil {
		log.WithError(err).Error("failed to initialize belt drive")
		return
	}
	log.Debug("belt drive initialized")

	// Setup signal handler for graceful shutdown on CTRL-C
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

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

		log.Debug("stopping back motor")
		if stopErr := back.Stop(config.Get().Mindstorm.EV3.DefaultStopAction); stopErr != nil {
			log.WithError(stopErr).Error("failed to stop back motor")
		}
		log.Debug("back motor stopped")
	}()

	// Run motor tests if debug mode is enabled
	if debug {
		testDuration := motorCfg.MotorTestTime

		// Test belt drive forward
		log.Info("starting belt drive forward test")
		if err := drive.Drive(motorCfg.Left.Speed); err != nil {
			log.WithError(err).Error("failed to start belt drive forward")
			return
		}
		time.Sleep(testDuration)

		if err := drive.Stop(); err != nil {
			log.WithError(err).Error("failed to stop belt drive forward")
		}
		log.Info("belt drive forward test complete")

		// Test belt drive backwards
		log.Info("starting belt drive backwards test")
		if err := drive.Drive(-motorCfg.Left.Speed); err != nil {
			log.WithError(err).Error("failed to start belt drive backwards")
			return
		}
		time.Sleep(testDuration)

		if err := drive.Stop(); err != nil {
			log.WithError(err).Error("failed to stop belt drive backwards")
		}
		log.Info("belt drive backwards test complete")

		// Test 360-degree clockwise rotation
		log.Info("starting 360 degree clockwise rotation test")

		leftRotateSpeed := int(float64(left.MaxSpeedTPS()) * motorCfg.Left.Speed)
		rightRotateSpeed := int(float64(right.MaxSpeedTPS()) * -motorCfg.Right.Speed)

		if err := left.RunForever(leftRotateSpeed); err != nil {
			log.WithError(err).Error("failed to start left belt for clockwise rotation")
			return
		}

		if err := right.RunForever(rightRotateSpeed); err != nil {
			log.WithError(err).Error("failed to start right belt for clockwise rotation")
			return
		}

		time.Sleep(testDuration)

		if err := drive.Stop(); err != nil {
			log.WithError(err).Error("failed to stop after clockwise rotation")
		}
		log.Info("360 degree clockwise rotation test complete")

		// Test 360-degree counter-clockwise rotation
		log.Info("starting 360 degree counter-clockwise rotation test")

		leftRotateSpeed = int(float64(left.MaxSpeedTPS()) * -motorCfg.Left.Speed)
		rightRotateSpeed = int(float64(right.MaxSpeedTPS()) * motorCfg.Right.Speed)

		if err := left.RunForever(leftRotateSpeed); err != nil {
			log.WithError(err).Error("failed to start left belt for counter-clockwise rotation")
			return
		}

		if err := right.RunForever(rightRotateSpeed); err != nil {
			log.WithError(err).Error("failed to start right belt for counter-clockwise rotation")
			return
		}

		time.Sleep(testDuration)

		if err := drive.Stop(); err != nil {
			log.WithError(err).Error("failed to stop after counter-clockwise rotation")
		}
		log.Info("360 degree counter-clockwise rotation test complete")

		// Test head motor
		log.Info("starting head motor test")
		headSpeed := int(float64(head.MaxSpeedTPS()) * motorCfg.Head.Speed)
		if err := head.RunTimed(headSpeed, int(testDuration.Milliseconds())); err != nil {
			log.WithError(err).Error("failed to start head motor")
			return
		}

		time.Sleep(testDuration)
		log.Info("head motor test complete")

		// Test back motor
		log.Info("starting back motor test")
		backSpeed := int(float64(back.MaxSpeedTPS()) * motorCfg.Back.Speed)
		if err := back.RunTimed(backSpeed, int(testDuration.Milliseconds())); err != nil {
			log.WithError(err).Error("failed to start back motor")
			return
		}

		time.Sleep(testDuration)
		log.Info("back motor forward test complete")

		// Test back motor inverted/reverse direction
		log.Info("starting back motor inverted test")
		invertedBackSpeed := -backSpeed
		if err := back.RunTimed(invertedBackSpeed, int(testDuration.Milliseconds())); err != nil {
			log.WithError(err).Error("failed to start back motor inverted")
			return
		}

		time.Sleep(testDuration)
		log.WithFields(log.Fields{
			"speed_tps":           invertedBackSpeed,
			"configured_speed":    motorCfg.Back.Speed,
			"configured_inverted": motorCfg.Back.Inverted,
		}).Info("back motor inverted test complete")
	} else {
		log.Info("skipping motor tests (enable with --debug)")
	}

	// Run head motor continuously until interrupted
	log.Info("starting continuous head motor operation (press CTRL-C to stop)")
	headSpeed := int(float64(head.MaxSpeedTPS()) * motorCfg.Head.Speed)
	if err := head.RunForever(headSpeed); err != nil {
		log.WithError(err).Error("failed to start head motor for continuous operation")
		return
	}
	log.WithField("speed_tps", headSpeed).Info("head motor running continuously")

	// Start the drive command server. The PC (vision program) connects to this
	// address and streams "<throttle> <turn>" commands that steer the belts
	// toward the white balls. This runs until interrupted.
	server := mindstorm.NewCommandServer(drive, config.Get().Mindstorm.EV3.ListenAddress)
	go func() {
		if err := server.ListenAndServe(); err != nil {
			log.WithError(err).Error("drive command server stopped")
			sigChan <- syscall.SIGTERM
		}
	}()
	log.WithField("address", config.Get().Mindstorm.EV3.ListenAddress).
		Info("waiting for PC navigation commands")

	// Wait for interrupt signal
	<-sigChan
	log.Info("interrupt signal received, shutting down")
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
