#include "ModbusRTU.h"
#include "CRC16.h"

namespace
{
    constexpr uint8_t FC_READ_COILS = 0x01;
    constexpr uint8_t FC_READ_DISCRETE_INPUTS = 0x02;
    constexpr uint8_t FC_READ_HOLDING_REGISTERS = 0x03;
    constexpr uint8_t FC_READ_INPUT_REGISTERS = 0x04;
    constexpr uint8_t FC_WRITE_SINGLE_COIL = 0x05;
    constexpr uint8_t FC_WRITE_SINGLE_REGISTER = 0x06;
    constexpr uint8_t FC_WRITE_MULTIPLE_REGISTERS = 0x10;
}

ModbusRTU::ModbusRTU(HardwareSerial& serial)
    : _serial(&serial)
{
}

void ModbusRTU::begin(uint32_t baudrate, uint32_t config, int16_t dePin)
{
    _serial->begin(baudrate, config);

    _dePin = dePin;
    if (_dePin >= 0)
    {
        pinMode(_dePin, OUTPUT);
        digitalWrite(_dePin, LOW);
    }

    // Modbus RTU inter-frame silence: 3.5 characters (11 bits each),
    // fixed 1750 us above 19200 baud per the specification.
    _interFrameMicros = (baudrate > 19200) ? 1750 : (38500000UL / baudrate);
}

ModbusResult ModbusRTU::readCoils(uint8_t slaveId, uint16_t address, uint16_t count, bool* values)
{
    return readBits(FC_READ_COILS, slaveId, address, count, values);
}

ModbusResult ModbusRTU::readDiscreteInputs(uint8_t slaveId, uint16_t address, uint16_t count, bool* values)
{
    return readBits(FC_READ_DISCRETE_INPUTS, slaveId, address, count, values);
}

ModbusResult ModbusRTU::readHoldingRegisters(uint8_t slaveId, uint16_t address, uint16_t count, uint16_t* values)
{
    return readRegisters(FC_READ_HOLDING_REGISTERS, slaveId, address, count, values);
}

ModbusResult ModbusRTU::readInputRegisters(uint8_t slaveId, uint16_t address, uint16_t count, uint16_t* values)
{
    return readRegisters(FC_READ_INPUT_REGISTERS, slaveId, address, count, values);
}

ModbusResult ModbusRTU::readHoldingRegister(uint8_t slaveId, uint16_t address, uint16_t& value)
{
    return readRegisters(FC_READ_HOLDING_REGISTERS, slaveId, address, 1, &value);
}

ModbusResult ModbusRTU::readInputRegister(uint8_t slaveId, uint16_t address, uint16_t& value)
{
    return readRegisters(FC_READ_INPUT_REGISTERS, slaveId, address, 1, &value);
}

ModbusResult ModbusRTU::writeCoil(uint8_t slaveId, uint16_t address, bool state)
{
    const uint16_t value = state ? 0xFF00 : 0x0000;

    _frame[0] = slaveId;
    _frame[1] = FC_WRITE_SINGLE_COIL;
    _frame[2] = address >> 8;
    _frame[3] = address & 0xFF;
    _frame[4] = value >> 8;
    _frame[5] = value & 0xFF;

    // Reply echoes the request: id + fc + addr + value + crc = 8 bytes
    return transaction(6, 8);
}

ModbusResult ModbusRTU::writeHoldingRegister(uint8_t slaveId, uint16_t address, uint16_t value)
{
    _frame[0] = slaveId;
    _frame[1] = FC_WRITE_SINGLE_REGISTER;
    _frame[2] = address >> 8;
    _frame[3] = address & 0xFF;
    _frame[4] = value >> 8;
    _frame[5] = value & 0xFF;

    return transaction(6, 8);
}

ModbusResult ModbusRTU::writeHoldingRegisters(uint8_t slaveId, uint16_t address, const uint16_t* values, uint16_t count)
{
    if (count == 0 || count > MAX_REGISTER_COUNT || values == nullptr)
    {
        return ModbusResult::InvalidArgument;
    }

    _frame[0] = slaveId;
    _frame[1] = FC_WRITE_MULTIPLE_REGISTERS;
    _frame[2] = address >> 8;
    _frame[3] = address & 0xFF;
    _frame[4] = count >> 8;
    _frame[5] = count & 0xFF;
    _frame[6] = count * 2;

    for (uint16_t i = 0; i < count; i++)
    {
        _frame[7 + i * 2] = values[i] >> 8;
        _frame[8 + i * 2] = values[i] & 0xFF;
    }

    // Reply: id + fc + addr + count + crc = 8 bytes
    return transaction(7 + count * 2, 8);
}

ModbusResult ModbusRTU::readBits(uint8_t functionCode, uint8_t slaveId, uint16_t address, uint16_t count, bool* values)
{
    if (count == 0 || count > MAX_BIT_COUNT || values == nullptr)
    {
        return ModbusResult::InvalidArgument;
    }

    _frame[0] = slaveId;
    _frame[1] = functionCode;
    _frame[2] = address >> 8;
    _frame[3] = address & 0xFF;
    _frame[4] = count >> 8;
    _frame[5] = count & 0xFF;

    // Reply: id + fc + byteCount + data + crc
    const size_t dataBytes = (count + 7) / 8;
    ModbusResult result = transaction(6, 5 + dataBytes);
    if (result != ModbusResult::Success)
    {
        return result;
    }

    for (uint16_t i = 0; i < count; i++)
    {
        values[i] = (_frame[3 + i / 8] >> (i % 8)) & 0x01;
    }

    return ModbusResult::Success;
}

ModbusResult ModbusRTU::readRegisters(uint8_t functionCode, uint8_t slaveId, uint16_t address, uint16_t count, uint16_t* values)
{
    if (count == 0 || count > MAX_REGISTER_COUNT || values == nullptr)
    {
        return ModbusResult::InvalidArgument;
    }

    _frame[0] = slaveId;
    _frame[1] = functionCode;
    _frame[2] = address >> 8;
    _frame[3] = address & 0xFF;
    _frame[4] = count >> 8;
    _frame[5] = count & 0xFF;

    ModbusResult result = transaction(6, 5 + count * 2);
    if (result != ModbusResult::Success)
    {
        return result;
    }

    if (_frame[2] != count * 2)
    {
        return ModbusResult::InvalidResponse;
    }

    for (uint16_t i = 0; i < count; i++)
    {
        values[i] = (static_cast<uint16_t>(_frame[3 + i * 2]) << 8) | _frame[4 + i * 2];
    }

    return ModbusResult::Success;
}

ModbusResult ModbusRTU::transaction(size_t frameLength, size_t expectedLength)
{
    const uint8_t slaveId = _frame[0];
    const uint8_t functionCode = _frame[1];

    const uint16_t crc = CRC16::calculate(_frame, frameLength);
    _frame[frameLength] = crc & 0xFF; // Modbus RTU sends CRC low byte first
    _frame[frameLength + 1] = crc >> 8;

    // Respect the 3.5 character inter-frame silence since the last bus activity
    while (static_cast<uint32_t>(micros() - _lastActivityMicros) < _interFrameMicros)
    {
        yield();
    }

    // Drop any stale bytes (noise, late replies from a previous timeout)
    while (_serial->available())
    {
        _serial->read();
    }

    if (_dePin >= 0)
    {
        digitalWrite(_dePin, HIGH);
    }

    _serial->write(_frame, frameLength + 2);
    _serial->flush(); // blocks until the last stop bit left the wire

    if (_dePin >= 0)
    {
        digitalWrite(_dePin, LOW);
    }

    _lastActivityMicros = micros();

    // Broadcast requests (id 0) are never answered
    if (slaveId == 0)
    {
        return ModbusResult::Success;
    }

    size_t received = 0;
    size_t expected = expectedLength;
    const uint32_t start = millis();

    while (received < expected)
    {
        if (_serial->available())
        {
            _frame[received++] = _serial->read();

            // Exception replies are always 5 bytes: id, fc|0x80, code, crc
            if (received == 2 && (_frame[1] & 0x80))
            {
                expected = 5;
            }
        }
        else if (millis() - start > _timeoutMs)
        {
            _lastActivityMicros = micros();
            return ModbusResult::Timeout;
        }
        else
        {
            yield();
        }
    }

    _lastActivityMicros = micros();

    const uint16_t replyCrc = CRC16::calculate(_frame, expected - 2);
    if (_frame[expected - 2] != (replyCrc & 0xFF) || _frame[expected - 1] != (replyCrc >> 8))
    {
        return ModbusResult::CrcMismatch;
    }

    if (_frame[0] != slaveId)
    {
        return ModbusResult::InvalidResponse;
    }

    if (_frame[1] & 0x80)
    {
        return exceptionToResult(_frame[2]);
    }

    if (_frame[1] != functionCode)
    {
        return ModbusResult::InvalidResponse;
    }

    return ModbusResult::Success;
}

ModbusResult ModbusRTU::exceptionToResult(uint8_t exceptionCode)
{
    switch (exceptionCode)
    {
        case 0x01: return ModbusResult::ExceptionIllegalFunction;
        case 0x02: return ModbusResult::ExceptionIllegalAddress;
        case 0x03: return ModbusResult::ExceptionIllegalValue;
        case 0x04: return ModbusResult::ExceptionDeviceFailure;
        default:   return ModbusResult::ExceptionOther;
    }
}

const char* ModbusRTU::resultToString(ModbusResult result)
{
    switch (result)
    {
        case ModbusResult::Success:                  return "Success";
        case ModbusResult::Timeout:                  return "Timeout";
        case ModbusResult::CrcMismatch:              return "CRC mismatch";
        case ModbusResult::InvalidResponse:          return "Invalid response";
        case ModbusResult::InvalidArgument:          return "Invalid argument";
        case ModbusResult::ExceptionIllegalFunction: return "Exception: illegal function";
        case ModbusResult::ExceptionIllegalAddress:  return "Exception: illegal data address";
        case ModbusResult::ExceptionIllegalValue:    return "Exception: illegal data value";
        case ModbusResult::ExceptionDeviceFailure:   return "Exception: device failure";
        case ModbusResult::ExceptionOther:           return "Exception: other";
    }
    return "Unknown";
}
