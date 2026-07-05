#include "BLSD20Modbus.h"

using namespace BLSD20;

BLSD20Modbus::BLSD20Modbus(ModbusRTU& bus, uint8_t slaveId)
    : _bus(&bus)
    , _slaveId(slaveId)
{
}

// --- Connection --------------------------------------------------------------

bool BLSD20Modbus::ping()
{
    uint16_t hwMajor = 0;
    if (!readInputReg(IR_HW_MAJOR, hwMajor))
    {
        return false;
    }
    return hwMajor == HW_MAJOR_EXPECTED;
}

// --- Motion commands -----------------------------------------------------------

bool BLSD20Modbus::start()
{
    return writeCoil(COIL_START, true);
}

bool BLSD20Modbus::stop()
{
    return writeCoil(COIL_STOP, true);
}

bool BLSD20Modbus::emergencyStop()
{
    return writeCoil(COIL_HARD_STOP, true);
}

bool BLSD20Modbus::setSpeed(uint16_t rpm)
{
    return writeHolding(HR_SPEED, rpm);
}

bool BLSD20Modbus::setDirection(BLSD20Direction dir)
{
    return writeHolding(HR_DIRECTION, static_cast<uint16_t>(dir));
}

bool BLSD20Modbus::setAcceleration(uint16_t acc)
{
    return writeHolding(HR_ACC, acc);
}

bool BLSD20Modbus::setDeceleration(uint16_t dec)
{
    return writeHolding(HR_DEC, dec);
}

bool BLSD20Modbus::moveOffset(int32_t offset)
{
    return setRotationMode(BLSD20RotationMode::Offset)
        && writeHolding32(HR_OFFSET, offset)
        && start();
}

bool BLSD20Modbus::moveTo(int32_t position)
{
    return setPresetPosition(1, position)
        && moveToPreset(1);
}

bool BLSD20Modbus::moveToPreset(uint8_t presetNumber)
{
    if (presetNumber < 1 || presetNumber > 4)
    {
        _lastResult = ModbusResult::InvalidArgument;
        return false;
    }

    return setRotationMode(BLSD20RotationMode::Position)
        && writeHolding(HR_POSITION_N, presetNumber)
        && start();
}

bool BLSD20Modbus::setPresetPosition(uint8_t presetNumber, int32_t position)
{
    if (presetNumber < 1 || presetNumber > 4)
    {
        _lastResult = ModbusResult::InvalidArgument;
        return false;
    }

    // Presets are 32-bit registers two addresses apart (501Bh, 501Dh, ...)
    const uint16_t address = HR_TARGET_POSITION1 + (presetNumber - 1) * 2;
    return writeHolding32(address, position);
}

bool BLSD20Modbus::resetPosition()
{
    return writeCoil(COIL_CLR_POSITION, true);
}

// --- Configuration -------------------------------------------------------------

bool BLSD20Modbus::setControlMode(BLSD20ControlMode mode)
{
    return writeHolding(HR_MODE_DEVICE, static_cast<uint16_t>(mode));
}

bool BLSD20Modbus::setRotationMode(BLSD20RotationMode mode)
{
    return writeHolding(HR_MODE_ROTATION, static_cast<uint16_t>(mode));
}

bool BLSD20Modbus::setCurrentLimit(uint16_t milliamps)
{
    return writeHolding(HR_REF_CURRENT, milliamps);
}

bool BLSD20Modbus::setHoldCurrent(uint16_t milliamps)
{
    return writeHolding(HR_HOLD_CURRENT, milliamps);
}

bool BLSD20Modbus::setPoleCount(uint16_t poles)
{
    return writeHolding(HR_N_POLE, poles);
}

bool BLSD20Modbus::useExternalSpeedInput(bool enable)
{
    return writeHolding(HR_USE_EXTERN_SPEED, enable ? 2 : 1);
}

bool BLSD20Modbus::invertDirection(bool invert)
{
    return writeCoil(COIL_INV_DIR, invert);
}

bool BLSD20Modbus::invertPositionCount(bool invert)
{
    return writeCoil(COIL_INV_CNT, invert);
}

bool BLSD20Modbus::enableAcceleration(bool enable)
{
    return writeCoil(COIL_ACC_ON, enable);
}

bool BLSD20Modbus::enableDeceleration(bool enable)
{
    return writeCoil(COIL_DEC_ON, enable);
}

bool BLSD20Modbus::enableFourQuadrantRegulation(bool enable)
{
    return writeCoil(COIL_FQ_REGULATOR_ON, enable);
}

bool BLSD20Modbus::enableAutoHold(bool enable)
{
    return writeCoil(COIL_AUTO_HOLD_MODE, enable);
}

// --- Status / telemetry ----------------------------------------------------------

BLSD20Status BLSD20Modbus::getStatus()
{
    uint16_t value = 0;
    readInputReg(IR_STATUS, value);
    return static_cast<BLSD20Status>(value);
}

bool BLSD20Modbus::isMoving()
{
    uint16_t value = 0;
    if (!readInputReg(IR_STATUS, value))
    {
        return false;
    }
    return value != 0;
}

uint16_t BLSD20Modbus::getSpeed()
{
    uint16_t value = 0;
    readInputReg(IR_SPEED_VALID, value);
    return value;
}

uint16_t BLSD20Modbus::getCurrent()
{
    uint16_t value = 0;
    readInputReg(IR_CURRENT_VALID, value);
    return value;
}

int32_t BLSD20Modbus::getPosition()
{
    int32_t value = 0;
    readInput32(IR_CURRENT_POSITION, value);
    return value;
}

float BLSD20Modbus::getTemperatureMCU()
{
    uint16_t value = 0;
    readInputReg(IR_TEMPERATURE_MCU, value);
    return value / 10.0f;
}

float BLSD20Modbus::getTemperatureMosfet()
{
    uint16_t value = 0;
    readInputReg(IR_TEMPERATURE_MOSFET, value);
    return value / 10.0f;
}

float BLSD20Modbus::getTemperatureBrake()
{
    uint16_t value = 0;
    readInputReg(IR_TEMPERATURE_BRAKE, value);
    return value / 10.0f;
}

uint16_t BLSD20Modbus::getErrorFlags()
{
    uint16_t value = 0;
    readHolding(HR_ERROR, value);
    return value;
}

bool BLSD20Modbus::hasError()
{
    return getErrorFlags() != 0;
}

bool BLSD20Modbus::readInput1()
{
    return readDiscrete(DI_IN1);
}

bool BLSD20Modbus::readInput2()
{
    return readDiscrete(DI_IN2);
}

bool BLSD20Modbus::readHardStopInput()
{
    return readDiscrete(DI_HARD_STOP);
}

uint16_t BLSD20Modbus::getTargetSpeed()
{
    uint16_t value = 0;
    readHolding(HR_SPEED, value);
    return value;
}

// --- Persistence / system ---------------------------------------------------------

bool BLSD20Modbus::saveSettings()
{
    return writeHolding(HR_FLAG_SAVE_INI, MAGIC_SAVE_INI);
}

bool BLSD20Modbus::restart()
{
    return writeHolding(HR_FLAG_RESTART, MAGIC_RESTART);
}

bool BLSD20Modbus::writeSlaveAddress(uint8_t newAddress)
{
    if (newAddress > 247)
    {
        _lastResult = ModbusResult::InvalidArgument;
        return false;
    }
    return writeHolding(HR_SLAVE_ADDRESS, newAddress);
}

// --- Private helpers ---------------------------------------------------------------

bool BLSD20Modbus::writeCoil(uint16_t address, bool state)
{
    _lastResult = _bus->writeCoil(_slaveId, address, state);
    return _lastResult == ModbusResult::Success;
}

bool BLSD20Modbus::writeHolding(uint16_t address, uint16_t value)
{
    _lastResult = _bus->writeHoldingRegister(_slaveId, address, value);
    return _lastResult == ModbusResult::Success;
}

bool BLSD20Modbus::writeHolding32(uint16_t address, int32_t value)
{
    const uint32_t raw = static_cast<uint32_t>(value);
    uint16_t words[2];

    if (_wordOrder == BLSD20WordOrder::LowWordFirst)
    {
        words[0] = raw & 0xFFFF;
        words[1] = raw >> 16;
    }
    else
    {
        words[0] = raw >> 16;
        words[1] = raw & 0xFFFF;
    }

    _lastResult = _bus->writeHoldingRegisters(_slaveId, address, words, 2);
    return _lastResult == ModbusResult::Success;
}

bool BLSD20Modbus::readHolding(uint16_t address, uint16_t& value)
{
    _lastResult = _bus->readHoldingRegister(_slaveId, address, value);
    return _lastResult == ModbusResult::Success;
}

bool BLSD20Modbus::readInputReg(uint16_t address, uint16_t& value)
{
    _lastResult = _bus->readInputRegister(_slaveId, address, value);
    return _lastResult == ModbusResult::Success;
}

bool BLSD20Modbus::readInput32(uint16_t address, int32_t& value)
{
    uint16_t words[2] = {0, 0};
    _lastResult = _bus->readInputRegisters(_slaveId, address, 2, words);
    if (_lastResult != ModbusResult::Success)
    {
        return false;
    }

    uint32_t raw;
    if (_wordOrder == BLSD20WordOrder::LowWordFirst)
    {
        raw = (static_cast<uint32_t>(words[1]) << 16) | words[0];
    }
    else
    {
        raw = (static_cast<uint32_t>(words[0]) << 16) | words[1];
    }

    value = static_cast<int32_t>(raw);
    return true;
}

bool BLSD20Modbus::readDiscrete(uint16_t address)
{
    bool state = false;
    _lastResult = _bus->readDiscreteInputs(_slaveId, address, 1, &state);
    return _lastResult == ModbusResult::Success && state;
}
