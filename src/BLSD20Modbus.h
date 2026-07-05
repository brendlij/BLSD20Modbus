#pragma once

#include <Arduino.h>
#include "ModbusRTU.h"
#include "BLSD20Registers.h"

enum class BLSD20Direction : uint16_t
{
    Forward = 1,
    Backward = 2
};

enum class BLSD20ControlMode : uint16_t
{
    Modbus = 1,
    ExternalSignals = 2
};

enum class BLSD20RotationMode : uint16_t
{
    Continuous = 1, // run until stop()
    Offset = 2,     // move by a relative offset
    Position = 3    // move to a preset absolute position
};

enum class BLSD20Status : uint16_t
{
    Stopped = 0,
    Forward = 1,
    Backward = 2
};

// Word order used by the controller for 32-bit values spanning two
// consecutive 16-bit registers. The manual does not state the order
// explicitly; LowWordFirst is the default. If getPosition() returns
// garbage on your device (huge jumps instead of small counts), switch
// to HighWordFirst.
enum class BLSD20WordOrder : uint8_t
{
    LowWordFirst,
    HighWordFirst
};

// High-level driver for one BLSD-20Modbus motor controller.
// Several motors can share one ModbusRTU bus instance:
//
//   ModbusRTU bus(Serial1);
//   BLSD20Modbus motor(bus, 1);
//
//   bus.begin(115200, SERIAL_8E1, DE_PIN); // factory default is 8E1!
//   motor.setSpeed(3000);
//   motor.start();
class BLSD20Modbus
{
public:
    BLSD20Modbus(ModbusRTU& bus, uint8_t slaveId = 1);

    // --- Connection ---------------------------------------------------------
    // Reads the hardware id register; true if a BLSD-20Modbus answers.
    bool ping();

    // --- Motion commands ----------------------------------------------------
    bool start();         // start rotation with the configured acceleration
    bool stop();          // stop with the configured deceleration
    bool emergencyStop(); // abrupt stop (HARD_STOP)

    bool setSpeed(uint16_t rpm);            // 35..14000 rpm
    bool setDirection(BLSD20Direction dir);
    bool setAcceleration(uint16_t acc);     // 10..1000 (10 = 100 rps^2, 1000 = 5000 rps^2)
    bool setDeceleration(uint16_t dec);     // 10..1000

    // Relative move: sets rotation mode Offset, writes the offset and starts.
    bool moveOffset(int32_t offset);

    // Absolute move: stores the target in preset 1, selects it and starts.
    bool moveTo(int32_t position);

    // Absolute move using a previously stored preset position (1..4).
    bool moveToPreset(uint8_t presetNumber);
    bool setPresetPosition(uint8_t presetNumber, int32_t position);

    bool resetPosition(); // set CURRENT_POSITION to zero

    // --- Configuration ------------------------------------------------------
    bool setControlMode(BLSD20ControlMode mode);   // Modbus or external signals
    bool setRotationMode(BLSD20RotationMode mode); // Continuous / Offset / Position
    bool setCurrentLimit(uint16_t milliamps);      // 1000..20000 mA
    bool setHoldCurrent(uint16_t milliamps);       // 0..1000 mA
    bool setPoleCount(uint16_t poles);             // 1..12
    bool useExternalSpeedInput(bool enable);       // speed from SPEED input instead of Modbus

    bool invertDirection(bool invert);
    bool invertPositionCount(bool invert);
    bool enableAcceleration(bool enable);
    bool enableDeceleration(bool enable);
    bool enableFourQuadrantRegulation(bool enable);
    bool enableAutoHold(bool enable);

    // --- Status / telemetry ---------------------------------------------------
    // Getters return 0 on communication failure; check lastResult() when in doubt.
    BLSD20Status getStatus();
    bool isMoving();
    uint16_t getSpeed();   // actual speed in rpm
    uint16_t getCurrent(); // motor current in mA
    int32_t getPosition(); // Hall sensor counts

    float getTemperatureMCU();    // deg C
    float getTemperatureMosfet(); // deg C
    float getTemperatureBrake();  // deg C

    uint16_t getErrorFlags();     // bit field, see BLSD20::ErrorFlag
    bool hasError();              // any error bit set?

    bool readInput1();
    bool readInput2();
    bool readHardStopInput();

    // Target speed as configured in the controller (holding register).
    uint16_t getTargetSpeed();

    // --- Persistence / system -------------------------------------------------
    // Saves holding registers 5000h..501Fh and coils 2004h..2009h to flash.
    bool saveSettings();

    // Reboots the controller. Needed to apply changed communication settings.
    bool restart();

    // Writes a new Modbus slave address (0..247). Takes effect only after
    // saveSettings() + restart(); this object keeps talking to the old id
    // until you call setSlaveId() with the new one.
    bool writeSlaveAddress(uint8_t newAddress);

    void setSlaveId(uint8_t slaveId) { _slaveId = slaveId; }
    uint8_t slaveId() const { return _slaveId; }

    void setWordOrder(BLSD20WordOrder order) { _wordOrder = order; }

    // Result of the most recent Modbus transaction of this motor.
    ModbusResult lastResult() const { return _lastResult; }
    const char* lastResultText() const { return ModbusRTU::resultToString(_lastResult); }

private:
    bool writeCoil(uint16_t address, bool state);
    bool writeHolding(uint16_t address, uint16_t value);
    bool writeHolding32(uint16_t address, int32_t value);
    bool readHolding(uint16_t address, uint16_t& value);
    bool readInputReg(uint16_t address, uint16_t& value);
    bool readInput32(uint16_t address, int32_t& value);
    bool readDiscrete(uint16_t address);

    ModbusRTU* _bus;
    uint8_t _slaveId;
    BLSD20WordOrder _wordOrder = BLSD20WordOrder::LowWordFirst;
    ModbusResult _lastResult = ModbusResult::Success;
};
