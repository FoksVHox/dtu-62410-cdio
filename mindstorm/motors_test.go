package mindstorm

import (
	"os"
	"path/filepath"
	"testing"
)

func TestNewMotorAndRunForever(t *testing.T) {
	base := t.TempDir()
	motor0 := filepath.Join(base, "motor0")
	if err := os.MkdirAll(motor0, 0o755); err != nil {
		t.Fatalf("mkdir: %v", err)
	}

	write := func(name, value string) {
		t.Helper()
		if err := os.WriteFile(filepath.Join(motor0, name), []byte(value), 0o644); err != nil {
			t.Fatalf("write %s: %v", name, err)
		}
	}

	write("address", "outA\n")
	write("driver_name", "lego-ev3-l-motor\n")
	write("max_speed", "1050\n")
	write("speed_sp", "0\n")
	write("command", "\n")

	m, err := NewMotor(MotorConfig{
		BasePath:   base,
		Address:    "outA",
		DriverName: "lego-ev3-l-motor",
	})
	if err != nil {
		t.Fatalf("new motor: %v", err)
	}

	if err := m.RunForever(3000); err != nil {
		t.Fatalf("run forever: %v", err)
	}

	speedSP, err := os.ReadFile(filepath.Join(motor0, "speed_sp"))
	if err != nil {
		t.Fatalf("read speed_sp: %v", err)
	}
	if string(speedSP) != "1050" {
		t.Fatalf("unexpected speed_sp: %q", string(speedSP))
	}

	command, err := os.ReadFile(filepath.Join(motor0, "command"))
	if err != nil {
		t.Fatalf("read command: %v", err)
	}
	if string(command) != "run-forever" {
		t.Fatalf("unexpected command: %q", string(command))
	}
}

func TestBeltDriveMixing(t *testing.T) {
	left := &Motor{maxSpeedTPS: 1000, path: t.TempDir(), address: "outA"}
	right := &Motor{maxSpeedTPS: 1000, path: t.TempDir(), address: "outB"}

	for _, m := range []*Motor{left, right} {
		for _, name := range []string{"speed_sp", "command", "stop_action"} {
			if err := os.WriteFile(filepath.Join(m.path, name), []byte(""), 0o644); err != nil {
				t.Fatalf("seed %s: %v", name, err)
			}
		}
	}

	drive, err := NewBeltDrive(left, right)
	if err != nil {
		t.Fatalf("new belt drive: %v", err)
	}

	if err := drive.SetThrottle(0.5, 0.25); err != nil {
		t.Fatalf("set throttle: %v", err)
	}

	leftSP, err := os.ReadFile(filepath.Join(left.path, "speed_sp"))
	if err != nil {
		t.Fatalf("read left speed_sp: %v", err)
	}
	rightSP, err := os.ReadFile(filepath.Join(right.path, "speed_sp"))
	if err != nil {
		t.Fatalf("read right speed_sp: %v", err)
	}

	if string(leftSP) != "750" {
		t.Fatalf("unexpected left speed_sp: %q", string(leftSP))
	}
	if string(rightSP) != "250" {
		t.Fatalf("unexpected right speed_sp: %q", string(rightSP))
	}
}

