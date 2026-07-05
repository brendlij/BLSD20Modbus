#pragma once

#include <Arduino.h>

// Result of a Modbus transaction. Everything except Success means the
// request failed and any output values are left untouched.
enum class ModbusResult : uint8_t
{
    Success = 0,
    Timeout,             // no (complete) reply within the configured timeout
    CrcMismatch,         // reply received but CRC check failed
    InvalidResponse,     // reply from wrong slave / wrong function code
    InvalidArgument,     // bad count / buffer passed by the caller
    ExceptionIllegalFunction, // device exception 0x01
    ExceptionIllegalAddress,  // device exception 0x02
    ExceptionIllegalValue,    // device exception 0x03
    ExceptionDeviceFailure,   // device exception 0x04
    ExceptionOther            // any other device exception code
};

// Minimal Modbus RTU master for half-duplex RS-485 buses.
// One instance per bus; the slave id is passed per request, so several
// devices (e.g. several BLSD-20 controllers) can share one instance.
class ModbusRTU
{
public:
    explicit ModbusRTU(HardwareSerial& serial);

    // BLSD-20Modbus factory default is 115200 baud, 8 data bits, even
    // parity, 1 stop bit (SERIAL_8E1).
    // dePin: driver-enable pin of the RS-485 transceiver (DE/!RE tied
    // together). Pass -1 if the transceiver switches automatically or if
    // you use the Teensy hardware support (serial.transmitterEnable(pin)).
    void begin(uint32_t baudrate = 115200, uint32_t config = SERIAL_8E1, int16_t dePin = -1);

    void setTimeout(uint32_t timeoutMs) { _timeoutMs = timeoutMs; }

    ModbusResult readCoils(uint8_t slaveId, uint16_t address, uint16_t count, bool* values);
    ModbusResult readDiscreteInputs(uint8_t slaveId, uint16_t address, uint16_t count, bool* values);
    ModbusResult readHoldingRegisters(uint8_t slaveId, uint16_t address, uint16_t count, uint16_t* values);
    ModbusResult readInputRegisters(uint8_t slaveId, uint16_t address, uint16_t count, uint16_t* values);

    ModbusResult writeCoil(uint8_t slaveId, uint16_t address, bool state);
    ModbusResult writeHoldingRegister(uint8_t slaveId, uint16_t address, uint16_t value);
    ModbusResult writeHoldingRegisters(uint8_t slaveId, uint16_t address, const uint16_t* values, uint16_t count);

    // Convenience single-register overloads
    ModbusResult readHoldingRegister(uint8_t slaveId, uint16_t address, uint16_t& value);
    ModbusResult readInputRegister(uint8_t slaveId, uint16_t address, uint16_t& value);

    static const char* resultToString(ModbusResult result);

private:
    static constexpr size_t MAX_REGISTER_COUNT = 16; // enough for this library, keeps buffers small
    static constexpr size_t MAX_BIT_COUNT = 64;

    ModbusResult readBits(uint8_t functionCode, uint8_t slaveId, uint16_t address, uint16_t count, bool* values);
    ModbusResult readRegisters(uint8_t functionCode, uint8_t slaveId, uint16_t address, uint16_t count, uint16_t* values);

    // Sends _frame (frameLength bytes, CRC appended internally) and receives
    // the reply into _frame. expectedLength includes the CRC bytes.
    ModbusResult transaction(size_t frameLength, size_t expectedLength);

    static ModbusResult exceptionToResult(uint8_t exceptionCode);

    HardwareSerial* _serial;
    int16_t _dePin = -1;
    uint32_t _timeoutMs = 200;
    uint32_t _interFrameMicros = 1750;
    uint32_t _lastActivityMicros = 0;
    uint8_t _frame[64];
};
