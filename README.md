# BLSD20Modbus

Arduino/PlatformIO library for the **Smart Motor Devices BLSD-20Modbus** brushless DC motor controller, controlled over **RS-485 / Modbus RTU**.

Hardware-tested on a Teensy 4.1; the examples are additionally compiled in CI for ESP32 and Arduino Mega. The library only depends on `HardwareSerial`, so it runs on any Arduino-compatible board with a spare hardware serial port that supports **even parity** (Teensy, ESP32, Arduino Mega, STM32, SAMD, RP2040, ...). `SoftwareSerial` cannot generate even parity and is therefore **not** supported — the controller ships configured for 8E1.

## Features

- Modbus RTU master (function codes 01, 02, 03, 04, 05, 06, 16) with CRC check, timeouts and exception decoding
- High-level motor API: `start()`, `stop()`, `setSpeed()`, `moveTo()`, `moveOffset()`, ...
- Full register map of the BLSD-20Modbus as named constants (`BLSD20Registers.h`)
- 32-bit register support (position, offset, targets)
- Error register decoding (`BLSD20::ErrorFlag`)
- Multiple motors on one RS-485 bus (one `ModbusRTU`, several `BLSD20Modbus` instances)
- RS-485 driver-enable (DE/!RE) pin handling built in

## Quick start

```cpp
#include <BLSD20Modbus.h>

ModbusRTU bus(Serial1);        // one bus per RS-485 interface
BLSD20Modbus motor(bus, 1);    // slave id 1 = factory default

void setup()
{
    // Factory defaults of the controller: 115200 baud, 8 data bits,
    // EVEN parity, 1 stop bit  ->  SERIAL_8E1, not 8N1!
    bus.begin(115200, SERIAL_8E1, /* DE pin */ 2);

    motor.setControlMode(BLSD20ControlMode::Modbus);
    motor.setRotationMode(BLSD20RotationMode::Continuous);
    motor.setCurrentLimit(3000); // mA
    motor.setSpeed(1500);        // rpm
    motor.start();
}

void loop()
{
    Serial.println(motor.getSpeed());   // actual rpm
    Serial.println(motor.getPosition());// Hall counts
    delay(500);
}
```

See the [`examples/`](examples) folder: `BasicControl` (start/stop/speed) and
`PositionControl` (relative moves).

## Wiring

RS-485 is half-duplex, so the transceiver's transmit/receive direction has to
be switched. There are two common cases:

**Auto-direction transceiver** (e.g. Waveshare isolated TTL-to-RS485 converter) —
nothing to switch, pass `-1` as the DE pin:

| MCU serial | Converter |
|------------|-----------|
| TX (e.g. TX1) | TXD |
| RX (e.g. RX1) | RXD |
| 3.3V / GND | VCC / GND (logic side) |
| — | A/B + GND → controller RJ12 (manual Fig. 4) |

**Plain MAX485-style transceiver** — drive DE/!RE from a GPIO:

| MCU | Transceiver (MAX485) |
|-----|----------------------|
| TX  | DI |
| RX  | RO |
| DE pin (any GPIO) | DE + !RE (tied together) |
| — | A/B → controller RJ12 (manual Fig. 4) |

On Teensy you can alternatively let the hardware handle it:
`Serial1.transmitterEnable(pin);` before `bus.begin(...)` and pass `-1` as the DE pin.

> The Teensy 4.1 is not 5V-tolerant — power the transceiver's logic side from
> 3.3V (or use a level shifter / an isolated converter).

### Tested setup

Verified with a **Teensy 4.1** driving a **Waveshare "TTL TO RS485 (B)"**
industrial isolated converter (Serial1 → TTL side, RS485 side → controller):

- Galvanically isolated — protects the MCU/USB side from the motor supply.
- Automatic direction control (half-duplex) → use `DE_PIN = -1`, no GPIO needed.
- 3.3–5 V TTL compatible → drive it straight from the Teensy's 3.3 V logic.
- Built-in switchable 120 Ω termination — enable it at the bus end instead of
  (or in addition to) the controller's `RT` jumper for a clean signal on
  longer cables.

Note: on this converter the TTL pins are labelled from the *converter's* point
of view, so on some units you connect Teensy TX → converter RXD and
Teensy RX → converter TXD. If `ping()` times out, swap the two TTL wires
(and check the A/B pair on the RS485 side).

## API overview

### Motion
| Method | Description |
|---|---|
| `start()` / `stop()` | start/stop with configured acc/dec |
| `emergencyStop()` | abrupt stop (HARD_STOP) |
| `setSpeed(rpm)` | 35–14000 rpm |
| `setDirection(BLSD20Direction::Forward/Backward)` | |
| `setAcceleration(v)` / `setDeceleration(v)` | 10–1000 |
| `moveOffset(counts)` | relative move (rotation mode Offset) |
| `moveTo(position)` | absolute move (uses preset 1) |
| `setPresetPosition(n, pos)` / `moveToPreset(n)` | presets 1–4 |
| `resetPosition()` | zero the position counter |

### Status
| Method | Description |
|---|---|
| `getStatus()` | Stopped / Forward / Backward |
| `isMoving()` | |
| `getSpeed()` | actual rpm |
| `getCurrent()` | motor current in mA |
| `getPosition()` | 32-bit Hall counter |
| `getTemperatureMCU()` / `...Mosfet()` / `...Brake()` | °C |
| `getErrorFlags()` / `hasError()` | bits see `BLSD20::ErrorFlag` |
| `readInput1()` / `readInput2()` / `readHardStopInput()` | digital inputs |
| `ping()` | checks HW id register (expects 1000) |

### Configuration & system
| Method | Description |
|---|---|
| `setControlMode(...)` | Modbus / external signals |
| `setCurrentLimit(mA)` / `setHoldCurrent(mA)` | |
| `setPoleCount(n)` | motor poles 1–12 |
| `useExternalSpeedInput(bool)` | speed via SPEED analog input |
| `invertDirection(bool)` / `invertPositionCount(bool)` | |
| `enableAutoHold(bool)`, `enableFourQuadrantRegulation(bool)` | |
| `saveSettings()` | persist registers to flash (FLAG_SAVE_INI) |
| `restart()` | reboot controller |
| `writeSlaveAddress(id)` | new Modbus id (active after save + restart) |

All commands return `bool`; on failure `lastResult()` / `lastResultText()` tell you why (timeout, CRC, Modbus exception, ...). Getters return `0` on communication failure.

## Notes & gotchas

- **Safety first**: set `setPoleCount()` to your motor and start with a low `setCurrentLimit()` (the examples use the 1000 mA minimum). A wrong pole count can make the controller run the motor away to full speed (manual section 6.2); a low current limit caps the torque so an untuned setup stays gentle. Keep a hardware `HARD STOP` / `emergencyStop()` within reach on first run.
- **Parity**: factory default is **EVEN** (`SERIAL_8E1`). With 8N1 the controller will never answer.
- **32-bit word order**: the manual does not specify the word order of 32-bit registers. Default is low-word-first; if `getPosition()` returns huge jumping values, call `motor.setWordOrder(BLSD20WordOrder::HighWordFirst);` once.
- **Communication settings** (id, baud rate, framing) only take effect after `saveSettings()` **and** `restart()`.
- Registers `START/STOP/HARD_STOP/CLR_POSITION` are self-resetting coils — the library just writes `1`.
- Wrong direction/counting setup can make the motor run away to max speed (manual section 6.2) — test with a current limit and `emergencyStop()` within reach.

## Installation

**PlatformIO** — add to `platformio.ini`:

```ini
lib_deps = https://github.com/brendlij/BLSD20Modbus.git
```

**Arduino IDE** — download the repository as a ZIP and use
*Sketch → Include Library → Add .ZIP Library*.

## Development

CI compiles the examples for Teensy 4.1, ESP32 and Arduino Mega on every push.
To build locally with PlatformIO:

```bash
pio ci --lib="." --board=teensy41 examples/BasicControl/BasicControl.ino
```

A local PlatformIO harness with an interactive serial console lives in `dev/`
(not part of the released package).

## License

MIT — see [LICENSE](LICENSE).
