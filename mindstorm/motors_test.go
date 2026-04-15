package mindstorm

import (
	"os"
	"path/filepath"
	"strings"
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

func TestNewMotorAddressAliases(t *testing.T) {
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

	write("address", "ev3-ports:outA\n")
	write("driver_name", "lego-ev3-l-motor\n")
	write("max_speed", "1050\n")

	for _, addr := range []string{"outA", "ev3-ports:outA", "motor0"} {
		m, err := NewMotor(MotorConfig{BasePath: base, Address: addr})
		if err != nil {
			t.Fatalf("new motor (%s): %v", addr, err)
		}
		if m.address != addr {
			t.Fatalf("unexpected stored address for %s: %q", addr, m.address)
		}
	}
}

func TestNewMotorErrorIncludesDiscoveredMotors(t *testing.T) {
	base := t.TempDir()
	motor0 := filepath.Join(base, "motor0")
	if err := os.MkdirAll(motor0, 0o755); err != nil {
		t.Fatalf("mkdir: %v", err)
	}

	if err := os.WriteFile(filepath.Join(motor0, "address"), []byte("ev3-ports:outA\n"), 0o644); err != nil {
		t.Fatalf("write address: %v", err)
	}
	if err := os.WriteFile(filepath.Join(motor0, "driver_name"), []byte("lego-ev3-l-motor\n"), 0o644); err != nil {
		t.Fatalf("write driver_name: %v", err)
	}
	if err := os.WriteFile(filepath.Join(motor0, "max_speed"), []byte("1050\n"), 0o644); err != nil {
		t.Fatalf("write max_speed: %v", err)
	}

	_, err := NewMotor(MotorConfig{BasePath: base, Address: "outB"})
	if err == nil {
		t.Fatal("expected error")
	}
	if !strings.Contains(err.Error(), "motor0(address=ev3-ports:outA") {
		t.Fatalf("error should include discovered motor details, got: %v", err)
	}
}

