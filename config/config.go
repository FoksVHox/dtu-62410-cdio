package config

import (
	"os"
	"sync"

	"github.com/creasty/defaults"
	"gopkg.in/yaml.v3"
)

const DefaultLocation = "./config.yml"

var (
	mu            sync.RWMutex
	_config       *Configuration
	_debugViaFlag bool
)

// Locker specific to writing the configuration to the disk, this happens
// in areas that might already be locked, so we don't want to crash the process.
var _writeLock sync.Mutex

// SystemConfiguration defines basic system configuration settings.
type SystemConfiguration struct {
	// The root directory where all of the data is stored at.
	RootDirectory string `default:"/var/lib/bot" json:"-" yaml:"root_directory"`

	// Directory where logs for bot events are logged.
	LogDirectory string `default:"/var/log/bot" json:"-" yaml:"log_directory"`
}

// EV3Configuration defines EV3dev sysfs settings for motors.
type EV3Configuration struct {
	// Path to the EV3dev tacho-motor class directory.
	MotorClassPath string `default:"/sys/class/tacho-motor" json:"motor_class_path" yaml:"motor_class_path"`

	// Default stop action when stopping motors (coast, brake, hold).
	DefaultStopAction string `default:"brake" json:"default_stop_action" yaml:"default_stop_action"`
}

// MotorConfiguration defines how a single motor should be discovered and controlled.
type MotorConfiguration struct {
	Address    string `json:"address" yaml:"address"`
	DriverName string `json:"driver_name" yaml:"driver_name"`
	Inverted   bool   `json:"inverted" yaml:"inverted"`
}

// MotorsConfiguration defines left/right motor mappings for the belt drive.
type MotorsConfiguration struct {
	Left  MotorConfiguration `json:"left" yaml:"left"`
	Right MotorConfiguration `json:"right" yaml:"right"`
}

// MindstormConfiguration defines defaults for the EV3 SDK package.
type MindstormConfiguration struct {
	EV3    EV3Configuration    `json:"ev3" yaml:"ev3"`
	Motors MotorsConfiguration `json:"motors" yaml:"motors"`
}

type Configuration struct {
	// The location from which this configuration instance was instantiated.
	path string

	// Determines if bot should be running in debug mode. This value is ignored
	// if the debug flag is passed through the command line arguments.
	Debug bool

	System    SystemConfiguration    `json:"system" yaml:"system"`
	Mindstorm MindstormConfiguration `json:"mindstorm" yaml:"mindstorm"`
}

// NewAtPath creates a new struct and set the path where it should be stored.
// This function does not modify the currently stored global configuration.
func NewAtPath(path string) (*Configuration, error) {
	var c Configuration
	// Configures the default values for many of the configuration options present
	// in the structs. Values set in the configuration file take priority over the
	// default values.
	if err := defaults.Set(&c); err != nil {
		return nil, err
	}
	// Track the location where we created this configuration.
	c.path = path
	return &c, nil
}

// Set the global configuration instance. This is a blocking operation such that
// anything trying to set a different configuration value, or read the configuration
// will be paused until it is complete.
func Set(c *Configuration) {
	mu.Lock()
	defer mu.Unlock()

	_config = c
}

// SetDebugViaFlag tracks if the application is running in debug mode because of
// a command line flag argument. If so we do not want to store that configuration
// change to the disk.
func SetDebugViaFlag(d bool) {
	mu.Lock()
	defer mu.Unlock()
	_config.Debug = d
	_debugViaFlag = d
}

// Get returns the global configuration instance. This is a thread-safe operation
// that will block if the configuration is presently being modified.
//
// Be aware that you CANNOT make modifications to the currently stored configuration
// by modifying the struct returned by this function. The only way to make
// modifications is by using the Update() function and passing data through in
// the callback.
func Get() *Configuration {
	mu.RLock()
	if _config == nil {
		mu.RUnlock()
		c, err := NewAtPath(DefaultLocation)
		if err != nil {
			return &Configuration{}
		}
		return c
	}

	// Create a copy of the struct so that all modifications made beyond this
	// point are immutable.
	//goland:noinspection GoVetCopyLock
	c := *_config
	mu.RUnlock()
	return &c
}

// FromFile reads the configuration from the provided file and stores it in the
// global singleton for this instance.
func FromFile(path string) error {
	b, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	c, err := NewAtPath(path)
	if err != nil {
		return err
	}

	if err := yaml.Unmarshal(b, c); err != nil {
		return err
	}

	// Store this configuration in the global state.
	Set(c)
	return nil
}
