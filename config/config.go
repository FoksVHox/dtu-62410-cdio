package config

import (
	"os"
	"sync"
	"time"

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
	Address    string  `json:"address" yaml:"address"`
	DriverName string  `json:"driver_name" yaml:"driver_name"`
	Inverted   bool    `json:"inverted" yaml:"inverted"`
	Speed      float64 `default:"0.5" json:"speed" yaml:"speed"`
}

// MotorsConfiguration defines left/right motor mappings for the belt drive.
type MotorsConfiguration struct {
	Left          MotorConfiguration `json:"left" yaml:"left"`
	Right         MotorConfiguration `json:"right" yaml:"right"`
	Head          MotorConfiguration `json:"head" yaml:"head"`
	Back          MotorConfiguration `json:"back" yaml:"back"`
	MotorTestTime time.Duration      `json:"motor_test_time" yaml:"motor_test_time"`
}

// MindstormConfiguration defines defaults for the EV3 SDK package.
type MindstormConfiguration struct {
	EV3    EV3Configuration    `json:"ev3" yaml:"ev3"`
	Motors MotorsConfiguration `json:"motors" yaml:"motors"`
}

// HSVBound holds a single HSV threshold value (H 0-179, S 0-255, V 0-255).
type HSVBound struct {
	H uint8 `json:"h" yaml:"h"`
	S uint8 `json:"s" yaml:"s"`
	V uint8 `json:"v" yaml:"v"`
}

// VisionConfiguration holds all GoCV / ball-detector parameters.
type VisionConfiguration struct {
	// Camera device index passed to OpenVideoCapture (0 = /dev/video0).
	CameraDevice int `default:"0" json:"camera_device" yaml:"camera_device"`
	// Capture resolution hints (0 = driver default).
	CameraWidth  int `default:"640" json:"camera_width" yaml:"camera_width"`
	CameraHeight int `default:"480" json:"camera_height" yaml:"camera_height"`
	CameraFPS    float64 `default:"30" json:"camera_fps" yaml:"camera_fps"`

	// HSV colour range for ball detection.
	// Ping-pong balls are white/light-yellow: low saturation, high value.
	HSVLower HSVBound `json:"hsv_lower" yaml:"hsv_lower"`
	HSVUpper HSVBound `json:"hsv_upper" yaml:"hsv_upper"`

	// Hough circle detection parameters.
	HoughDP        float64 `default:"1.2" json:"hough_dp" yaml:"hough_dp"`
	HoughMinDist   float64 `default:"30" json:"hough_min_dist" yaml:"hough_min_dist"`
	HoughParam1    float64 `default:"100" json:"hough_param1" yaml:"hough_param1"`
	HoughParam2    float64 `default:"20" json:"hough_param2" yaml:"hough_param2"`
	HoughMinRadius int     `default:"8" json:"hough_min_radius" yaml:"hough_min_radius"`
	HoughMaxRadius int     `default:"80" json:"hough_max_radius" yaml:"hough_max_radius"`

	// GaussianBlur kernel size (must be odd).
	BlurKernel int `default:"9" json:"blur_kernel" yaml:"blur_kernel"`

	// DistanceK is the empirical constant used in: distance = DistanceK / radius.
	// Increase this value if the robot stops too far from the ball.
	DistanceK float64 `default:"200" json:"distance_k" yaml:"distance_k"`

	// DebugVision draws detection circles onto a window when true.
	DebugVision bool `default:"false" json:"debug_vision" yaml:"debug_vision"`
}

// NavigationConfiguration holds high-level navigation / state-machine parameters.
type NavigationConfiguration struct {
	// DriveSpeed is the base forward throttle in [0, 1].
	DriveSpeed float64 `default:"0.35" json:"drive_speed" yaml:"drive_speed"`
	// TurnSpeed is the rotation throttle used while searching.
	TurnSpeed float64 `default:"0.25" json:"turn_speed" yaml:"turn_speed"`
	// SteeringGain scales the horizontal ball offset into a turn correction.
	SteeringGain float64 `default:"0.6" json:"steering_gain" yaml:"steering_gain"`
	// CollectDistanceThreshold: when EstimatedDistance drops below this value
	// the robot transitions to the collecting state.
	CollectDistanceThreshold float64 `default:"1.5" json:"collect_distance_threshold" yaml:"collect_distance_threshold"`
	// CollectDwellMs is how long (milliseconds) the collector motor runs during pickup.
	CollectDwellMs int `default:"1200" json:"collect_dwell_ms" yaml:"collect_dwell_ms"`
	// SearchTimeoutMs: after this many milliseconds without seeing a ball the
	// robot transitions back to searching.
	SearchTimeoutMs int `default:"2000" json:"search_timeout_ms" yaml:"search_timeout_ms"`
	// TickIntervalMs controls how fast the main navigation loop runs.
	TickIntervalMs int `default:"50" json:"tick_interval_ms" yaml:"tick_interval_ms"`
}

type Configuration struct {
	// The location from which this configuration instance was instantiated.
	path string

	// Determines if bot should be running in debug mode. This value is ignored
	// if the debug flag is passed through the command line arguments.
	Debug bool

	System     SystemConfiguration     `json:"system" yaml:"system"`
	Mindstorm  MindstormConfiguration  `json:"mindstorm" yaml:"mindstorm"`
	Vision     VisionConfiguration     `json:"vision" yaml:"vision"`
	Navigation NavigationConfiguration `json:"navigation" yaml:"navigation"`
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
	// Apply hard-coded HSV defaults that creasty/defaults cannot express as
	// struct tags because they are nested fields of a non-primitive type.
	c.Vision.HSVLower = HSVBound{H: 0, S: 0, V: 180}
	c.Vision.HSVUpper = HSVBound{H: 179, S: 60, V: 255}
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
