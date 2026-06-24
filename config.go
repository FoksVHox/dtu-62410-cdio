package main

import (
	"fmt"
	"os"

	"gopkg.in/yaml.v3"
)

// Config holds all run-time settings for the PC-side program.
type Config struct {
	Robot  RobotConfig  `yaml:"robot"`
	Camera CameraConfig `yaml:"camera"`
}

type RobotConfig struct {
	// Address is the TCP address of the EV3 command server, e.g. "169.254.201.177:9000".
	Address string `yaml:"address"`
}

type CameraConfig struct {
	// Device is the OS index of the webcam (0, 1, 2 …).
	Device int `yaml:"device"`
	Wait   int `yaml:"wait"`
}

// LoadConfig reads config.yml from the given path.
func LoadConfig(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("config: read %q: %w", path, err)
	}
	var cfg Config
	if err := yaml.Unmarshal(data, &cfg); err != nil {
		return nil, fmt.Errorf("config: parse %q: %w", path, err)
	}
	return &cfg, nil
}
