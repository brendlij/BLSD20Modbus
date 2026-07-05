# Changelog

All notable changes to this project are documented here.
The format is based on [Keep a Changelog](https://keepachangelog.com/).

## [0.1.0] - 2026-07-05

Initial release.

### Added
- `ModbusRTU` master for half-duplex RS-485: function codes 01, 02, 03, 04,
  05, 06, 16, with CRC-16 check, configurable timeout, inter-frame timing,
  Modbus exception decoding and optional DE/!RE driver-enable pin handling.
- `BLSD20Modbus` high-level motor driver: `start`, `stop`, `emergencyStop`,
  `setSpeed`, `setDirection`, `setAcceleration`/`setDeceleration`,
  `moveOffset`, `moveTo`, preset positions 1-4, `resetPosition`.
- Configuration: control/rotation mode, current and hold current limit,
  pole count, external speed input, direction/count inversion, auto-hold,
  four-quadrant regulation.
- Telemetry: status, actual speed, current, 32-bit position, MCU/MOSFET/brake
  temperatures, decoded error flags, digital inputs.
- Persistence: `saveSettings`, `restart`, `writeSlaveAddress`.
- Full BLSD-20Modbus register map as named constants in `BLSD20Registers.h`.
- Examples: `BasicControl`, `PositionControl`.
- Verified against real hardware on a Teensy 4.1.

### Not yet implemented
- User program feature (on-controller instruction VM, manual sections 6.6/6.9).
- Modbus ASCII transport (only RTU).
