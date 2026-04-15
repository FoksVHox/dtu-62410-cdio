# mindstorm SDK

This package provides a small EV3dev SDK for belt-driven robots using `tacho-motor` sysfs.

## What it includes

- Motor discovery by `address` (and optional `driver_name`), accepting `outA`, `ev3-ports:outA`, or sysfs dir names like `motor0`.
- Speed clamping based on EV3dev `max_speed`.
- Motor commands: `RunForever`, `RunTimed`, `Stop`, and `Reset`.
- Differential belt mixing with `BeltDrive` (`Drive`, `Turn`, `SetThrottle`, `Stop`).

## Example

```go
left, err := mindstorm.NewMotor(mindstorm.MotorConfig{Address: "outA", DriverName: "lego-ev3-l-motor"})
if err != nil {
	return err
}

right, err := mindstorm.NewMotor(mindstorm.MotorConfig{Address: "outB", DriverName: "lego-ev3-l-motor", Inverted: true})
if err != nil {
	return err
}

drive, err := mindstorm.NewBeltDrive(left, right)
if err != nil {
	return err
}

if err := drive.SetThrottle(0.6, 0.2); err != nil {
	return err
}

return drive.Stop()
```

## Test

```bash
go test ./...
```

