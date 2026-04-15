package mindstorm

import (
	"bot/config"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/apex/log"
)

// MotorConfig describes how a motor should be discovered and controlled on EV3dev.
type MotorConfig struct {
	BasePath   string
	Address    string
	DriverName string
	Inverted   bool
}

// Motor provides a thin SDK over an EV3dev tacho-motor sysfs entry.
type Motor struct {
	path        string
	address     string
	driverName  string
	maxSpeedTPS int // tacho counts/sec
	inverted    bool
}

// NewMotor discovers a motor in EV3dev by its address (for example "outA").
func NewMotor(cfg MotorConfig) (*Motor, error) {
	log.WithFields(log.Fields{
		"address":     cfg.Address,
		"driver_name": cfg.DriverName,
		"base_path":   cfg.BasePath,
		"inverted":    cfg.Inverted,
	}).Debug("mindstorm: initializing motor")

	if strings.TrimSpace(cfg.Address) == "" {
		return nil, fmt.Errorf("mindstorm: motor address is required")
	}

	basePath := cfg.BasePath
	if strings.TrimSpace(basePath) == "" {
		basePath = config.Get().Mindstorm.EV3.MotorClassPath
	}
	log.WithField("base_path", basePath).Debug("mindstorm: using motor class path")

	motorPath, err := findMotorPath(basePath, cfg.Address, cfg.DriverName)
	if err != nil {
		return nil, err
	}
	log.WithFields(log.Fields{"address": cfg.Address, "motor_path": motorPath}).Debug("mindstorm: discovered motor path")

	maxSpeedTPS, err := readIntAttr(motorPath, "max_speed")
	if err != nil {
		return nil, fmt.Errorf("mindstorm: read max speed: %w", err)
	}
	log.WithFields(log.Fields{"address": cfg.Address, "max_speed_tps": maxSpeedTPS}).Debug("mindstorm: loaded motor capabilities")

	return &Motor{
		path:        motorPath,
		address:     cfg.Address,
		driverName:  cfg.DriverName,
		maxSpeedTPS: maxSpeedTPS,
		inverted:    cfg.Inverted,
	}, nil
}

// MaxSpeedTPS returns the motor max speed as reported by EV3dev.
func (m *Motor) MaxSpeedTPS() int {
	return m.maxSpeedTPS
}

// RunForever sets speed_sp and starts the motor with run-forever.
func (m *Motor) RunForever(speedTPS int) error {
	if err := m.SetSpeedTPS(speedTPS); err != nil {
		return err
	}
	return m.writeAttr("command", "run-forever")
}

// RunTimed runs the motor for a fixed duration in milliseconds.
func (m *Motor) RunTimed(speedTPS, durationMS int) error {
	if durationMS <= 0 {
		return fmt.Errorf("mindstorm: duration must be greater than zero")
	}
	if err := m.SetSpeedTPS(speedTPS); err != nil {
		return err
	}
	if err := m.writeAttr("time_sp", strconv.Itoa(durationMS)); err != nil {
		return err
	}
	return m.writeAttr("command", "run-timed")
}

// SetSpeedTPS writes speed_sp after clamping to the valid range.
func (m *Motor) SetSpeedTPS(speedTPS int) error {
	speed := speedTPS
	if m.inverted {
		speed = -speed
	}

	if speed > m.maxSpeedTPS {
		speed = m.maxSpeedTPS
	}
	if speed < -m.maxSpeedTPS {
		speed = -m.maxSpeedTPS
	}

	return m.writeAttr("speed_sp", strconv.Itoa(speed))
}

// Stop stops the motor using the provided EV3dev stop action (coast, brake, hold).
func (m *Motor) Stop(action string) error {
	if strings.TrimSpace(action) == "" {
		action = "brake"
	}
	if err := m.writeAttr("stop_action", action); err != nil {
		return err
	}
	return m.writeAttr("command", "stop")
}

// Reset requests a motor reset from EV3dev.
func (m *Motor) Reset() error {
	return m.writeAttr("command", "reset")
}

func (m *Motor) writeAttr(name, value string) error {
	p := filepath.Join(m.path, name)
	if err := os.WriteFile(p, []byte(value), 0o644); err != nil {
		return fmt.Errorf("mindstorm: write %q for %s: %w", name, m.address, err)
	}
	return nil
}

func findMotorPath(basePath, address, driverName string) (string, error) {
	log.WithFields(log.Fields{
		"base_path":   basePath,
		"address":     address,
		"driver_name": driverName,
	}).Debug("mindstorm: scanning motor class path")

	entries, err := os.ReadDir(basePath)
	if err != nil {
		return "", fmt.Errorf("mindstorm: read motor class path %q: %w", basePath, err)
	}

	targetAddress := normalizeAddress(address)
	var discovered []string

	for _, entry := range entries {
		// EV3 sysfs class entries are commonly symlinks (for example motor0 -> ../../devices/...).
		// Skip only plain files and keep directories/symlinks as candidates.
		entryType := entry.Type()
		if !entry.IsDir() && entryType&os.ModeSymlink == 0 {
			log.WithField("entry", entry.Name()).Debug("mindstorm: skipping non-motor class entry")
			continue
		}
		motorPath := filepath.Join(basePath, entry.Name())

		motorAddress, err := readStringAttr(motorPath, "address")
		if err != nil {
			log.WithFields(log.Fields{"entry": entry.Name(), "motor_path": motorPath, "error": err.Error()}).Debug("mindstorm: candidate missing address attribute")
			continue
		}

		if name, nameErr := readStringAttr(motorPath, "driver_name"); nameErr == nil {
			discovered = append(discovered, fmt.Sprintf("%s(address=%s,driver=%s)", entry.Name(), motorAddress, name))
		} else {
			discovered = append(discovered, fmt.Sprintf("%s(address=%s)", entry.Name(), motorAddress))
		}
		log.WithFields(log.Fields{"entry": entry.Name(), "motor_address": motorAddress}).Debug("mindstorm: discovered motor candidate")

		discoveredAddress := normalizeAddress(motorAddress)
		if discoveredAddress != targetAddress && entry.Name() != address {
			log.WithFields(log.Fields{"entry": entry.Name(), "target": targetAddress, "candidate": discoveredAddress}).Debug("mindstorm: candidate does not match requested address")
			continue
		}

		if driverName == "" {
			log.WithFields(log.Fields{"entry": entry.Name(), "motor_path": motorPath}).Debug("mindstorm: matched motor by address")
			return motorPath, nil
		}

		name, err := readStringAttr(motorPath, "driver_name")
		if err != nil {
			log.WithFields(log.Fields{"entry": entry.Name(), "error": err.Error()}).Debug("mindstorm: candidate missing driver_name attribute")
			continue
		}
		log.WithFields(log.Fields{"entry": entry.Name(), "driver_name": name, "expected_driver": driverName}).Debug("mindstorm: comparing candidate driver")
		if name == driverName {
			log.WithFields(log.Fields{"entry": entry.Name(), "motor_path": motorPath}).Debug("mindstorm: matched motor by address and driver")
			return motorPath, nil
		}
	}

	if len(discovered) == 0 {
		discovered = append(discovered, "none")
	}

	if driverName == "" {
		return "", fmt.Errorf("mindstorm: motor at %q not found in %q (discovered: %s)", address, basePath, strings.Join(discovered, ", "))
	}
	return "", fmt.Errorf("mindstorm: motor at %q with driver %q not found in %q (discovered: %s)", address, driverName, basePath, strings.Join(discovered, ", "))
}

func normalizeAddress(address string) string {
	a := strings.TrimSpace(address)
	if i := strings.LastIndex(a, ":"); i != -1 {
		a = a[i+1:]
	}
	return a
}

func readIntAttr(dirPath, name string) (int, error) {
	s, err := readStringAttr(dirPath, name)
	if err != nil {
		return 0, err
	}
	v, err := strconv.Atoi(s)
	if err != nil {
		return 0, fmt.Errorf("mindstorm: parse %q from %s: %w", name, dirPath, err)
	}
	return v, nil
}

func readStringAttr(dirPath, name string) (string, error) {
	b, err := os.ReadFile(filepath.Join(dirPath, name))
	if err != nil {
		return "", fmt.Errorf("mindstorm: read %q from %s: %w", name, dirPath, err)
	}
	return strings.TrimSpace(string(b)), nil
}

// BeltDrive mixes throttle and turn values for a belt-driven differential robot.
type BeltDrive struct {
	left        *Motor
	right       *Motor
	maxSpeedTPS int
	stopAction  string
}

// NewBeltDrive creates a controller for two motors driving left and right belts.
func NewBeltDrive(left, right *Motor) (*BeltDrive, error) {
	if left == nil || right == nil {
		return nil, fmt.Errorf("mindstorm: both left and right motors are required")
	}

	maxSpeed := left.maxSpeedTPS
	if right.maxSpeedTPS < maxSpeed {
		maxSpeed = right.maxSpeedTPS
	}

	stopAction := config.Get().Mindstorm.EV3.DefaultStopAction
	if strings.TrimSpace(stopAction) == "" {
		stopAction = "brake"
	}

	return &BeltDrive{
		left:        left,
		right:       right,
		maxSpeedTPS: maxSpeed,
		stopAction:  stopAction,
	}, nil
}

// SetStopAction sets the action used by Stop() (coast, brake, hold).
func (d *BeltDrive) SetStopAction(action string) {
	if strings.TrimSpace(action) == "" {
		return
	}
	d.stopAction = action
}

// Drive sets forward or reverse throttle in the range [-1.0, 1.0].
func (d *BeltDrive) Drive(throttle float64) error {
	return d.SetThrottle(throttle, 0)
}

// Turn turns in place in the range [-1.0, 1.0].
func (d *BeltDrive) Turn(turn float64) error {
	return d.SetThrottle(0, turn)
}

// SetThrottle mixes throttle and turn into left and right motor speeds.
func (d *BeltDrive) SetThrottle(throttle, turn float64) error {
	leftMix := clampUnit(throttle + turn)
	rightMix := clampUnit(throttle - turn)

	leftSpeed := int(math.Round(leftMix * float64(d.maxSpeedTPS)))
	rightSpeed := int(math.Round(rightMix * float64(d.maxSpeedTPS)))
	log.WithFields(log.Fields{
		"throttle":        throttle,
		"turn":            turn,
		"left_speed_tps":  leftSpeed,
		"right_speed_tps": rightSpeed,
	}).Debug("mindstorm: applying mixed belt speeds")

	if err := d.left.RunForever(leftSpeed); err != nil {
		return err
	}
	if err := d.right.RunForever(rightSpeed); err != nil {
		return err
	}
	return nil
}

// Stop stops both belt motors with the configured stop action.
func (d *BeltDrive) Stop() error {
	if err := d.left.Stop(d.stopAction); err != nil {
		return err
	}
	if err := d.right.Stop(d.stopAction); err != nil {
		return err
	}
	return nil
}

func clampUnit(v float64) float64 {
	if v > 1.0 {
		return 1.0
	}
	if v < -1.0 {
		return -1.0
	}
	return v
}
