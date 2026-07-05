#pragma once

#include <stdint.h>

// Register map of the Smart Motor Devices BLSD-20Modbus controller.
// Addresses and values taken from the user manual BLSD.20.Modbus.001 (2025).
namespace BLSD20
{
    // --- Discrete inputs (function code 0x02), read-only -------------------
    constexpr uint16_t DI_IN1 = 0x1000;       // state of input IN1
    constexpr uint16_t DI_IN2 = 0x1001;       // state of input IN2
    constexpr uint16_t DI_HARD_STOP = 0x1002; // state of input HARD_STOP

    // --- Coils (function codes 0x01 / 0x05) --------------------------------
    // Writing 1 triggers the function; 0x2000..0x2003 self-reset afterwards.
    constexpr uint16_t COIL_START = 0x2000;           // start with set acceleration
    constexpr uint16_t COIL_STOP = 0x2001;            // stop with set deceleration
    constexpr uint16_t COIL_HARD_STOP = 0x2002;       // abrupt emergency stop
    constexpr uint16_t COIL_CLR_POSITION = 0x2003;    // reset CURRENT_POSITION to 0
    constexpr uint16_t COIL_INV_DIR = 0x2004;         // invert rotation direction
    constexpr uint16_t COIL_INV_CNT = 0x2005;         // invert position count direction
    constexpr uint16_t COIL_ACC_ON = 0x2006;          // enable acceleration (ACC register)
    constexpr uint16_t COIL_DEC_ON = 0x2007;          // enable deceleration (DEC register)
    constexpr uint16_t COIL_FQ_REGULATOR_ON = 0x2008; // four-quadrant speed regulation
    constexpr uint16_t COIL_AUTO_HOLD_MODE = 0x2009;  // automatic hold mode

    // --- Input registers (function code 0x04), read-only -------------------
    constexpr uint16_t IR_STATUS = 0x3000;           // 0 stop, 1 forward, 2 backward
    constexpr uint16_t IR_CURRENT_VALID = 0x3001;    // motor current in mA
    constexpr uint16_t IR_SPEED_VALID = 0x3002;      // actual speed in rpm
    constexpr uint16_t IR_CURRENT_POSITION = 0x3003; // 32-bit, Hall sensor counts
    constexpr uint16_t IR_TEMPERATURE_MCU = 0x3005;  // value/10 = deg C
    constexpr uint16_t IR_TEMPERATURE_MOSFET = 0x3006;
    constexpr uint16_t IR_TEMPERATURE_BRAKE = 0x3007;
    constexpr uint16_t IR_TASK_COUNTER = 0x3008;     // random value, link check
    constexpr uint16_t IR_STATUS_USER_PROGRAM = 0x3009;
    constexpr uint16_t IR_SPEED_INPUT_VALUE = 0x300A; // ADC value of SPEED input

    // --- Holding registers (function codes 0x03 / 0x06 / 0x10) -------------
    // RS-485 communication settings (active after save + restart)
    constexpr uint16_t HR_SLAVE_ADDRESS = 0x5000;     // 0..247, 0 = broadcast
    constexpr uint16_t HR_TYPE_MODBUS = 0x5001;       // framing, see manual 6.4
    constexpr uint16_t HR_BITRATE = 0x5002;           // baud rate index, see manual 6.4
    constexpr uint16_t HR_TIMEOUT_BROADCAST = 0x5003; // extra reply delay

    // Drive operation settings
    constexpr uint16_t HR_MODE_DEVICE = 0x5004;       // 1 Modbus, 2 external signals
    constexpr uint16_t HR_MODE_USER_PROGRAM = 0x5005; // 1 stop, 2 start, 3 autostart
    constexpr uint16_t HR_MODE_ROTATION = 0x5006;     // 1 continuous, 2 offset, 3 position
    constexpr uint16_t HR_MODE_EXT_IN = 0x5007;       // IN1/IN2 behaviour, see manual 6.5
    constexpr uint16_t HR_POSITION_N = 0x5008;        // preset position number 1..4
    constexpr uint16_t HR_REF_CURRENT = 0x5009;       // current limit, 1000..20000 mA
    constexpr uint16_t HR_HOLD_CURRENT = 0x500A;      // holding current, 0..1000 mA
    constexpr uint16_t HR_SPEED = 0x500B;             // target speed, 35..14000 rpm
    constexpr uint16_t HR_ACC = 0x500C;               // acceleration, 10..1000
    constexpr uint16_t HR_DEC = 0x500D;               // deceleration, 10..1000
    constexpr uint16_t HR_DIRECTION = 0x500E;         // 1 forward, 2 backward
    constexpr uint16_t HR_N_POLE = 0x500F;            // motor pole count, 1..12
    constexpr uint16_t HR_USE_EXTERN_SPEED = 0x5010;  // 1 Modbus, 2 SPEED input
    constexpr uint16_t HR_OFFSET_COMPENSATION = 0x5012;
    constexpr uint16_t HR_PRESSED_INPUTS_EXTERN = 0x5013; // debounce, ms
    constexpr uint16_t HR_WAITED_INPUTS_EXTERN = 0x5014;  // debounce, ms
    constexpr uint16_t HR_OFFSET = 0x5015;            // 32-bit, relative move
    constexpr uint16_t HR_OFFSET_CONST = 0x5017;      // 32-bit, managed by controller
    constexpr uint16_t HR_TARGET_POSITION = 0x5019;   // 32-bit, active target
    constexpr uint16_t HR_TARGET_POSITION1 = 0x501B;  // 32-bit, preset 1
    constexpr uint16_t HR_TARGET_POSITION2 = 0x501D;  // 32-bit, preset 2
    constexpr uint16_t HR_TARGET_POSITION3 = 0x501F;  // 32-bit, preset 3
    constexpr uint16_t HR_TARGET_POSITION4 = 0x5021;  // 32-bit, preset 4
    constexpr uint16_t HR_ERROR = 0x5023;             // error bit field
    constexpr uint16_t HR_FLAG_SAVE_INI = 0x5024;     // write MAGIC_SAVE_INI to save
    constexpr uint16_t HR_FLAG_SAVE_USER_PROGRAM = 0x5025;
    constexpr uint16_t HR_FLAG_RESTART = 0x5026;      // write MAGIC_RESTART to reboot
    constexpr uint16_t HR_PID_PROPORTIONAL = 0x5028;  // 32-bit
    constexpr uint16_t HR_PID_INTEGRAL = 0x502A;      // 32-bit
    constexpr uint16_t HR_PID_DIFFERENTIAL = 0x502C;  // 32-bit

    // User program access
    constexpr uint16_t HR_WRITE_CMD = 0x6000;
    constexpr uint16_t HR_CMD_W = 0x6001; // 32-bit
    constexpr uint16_t HR_READ_CMD = 0x6003;
    constexpr uint16_t HR_CMD_R = 0x6004; // 32-bit

    // System registers (user program data)
    constexpr uint16_t HR_AX_REG = 0x7000;
    constexpr uint16_t HR_PC_REG = 0x7006;

    // --- Identification input registers (function code 0x04) ---------------
    constexpr uint16_t IR_HW_MAJOR = 0x8001; // 1000 for BLSD-20Modbus
    constexpr uint16_t IR_HW_MINOR = 0x8002;
    constexpr uint16_t IR_FW_MAJOR = 0x8003;
    constexpr uint16_t IR_FW_MINOR = 0x8004;

    // --- Magic values -------------------------------------------------------
    constexpr uint16_t MAGIC_SAVE_INI = 0x37FA;
    constexpr uint16_t MAGIC_SAVE_USER_PROGRAM = 0x8426;
    constexpr uint16_t MAGIC_LOAD_USER_PROGRAM = 0x9346;
    constexpr uint16_t MAGIC_RESTART = 0x95AF;

    constexpr uint16_t HW_MAJOR_EXPECTED = 1000;

    // --- Bits of the ERROR register (5023h) ----------------------------------
    namespace ErrorFlag
    {
        constexpr uint16_t SupplyVoltageRange = 1u << 0;
        constexpr uint16_t WindingShortCircuit = 1u << 1;
        constexpr uint16_t BrakeOverheat = 1u << 2;
        constexpr uint16_t PowerOverheat = 1u << 3;
        constexpr uint16_t HallSensorError = 1u << 4;
        constexpr uint16_t EmergencyStop = 1u << 5;
        constexpr uint16_t McuOverheat = 1u << 6;
        constexpr uint16_t TestControlProgram = 1u << 7;
        constexpr uint16_t UserProgramError = 1u << 8;
        constexpr uint16_t SettingsReadWrite = 1u << 9;
        constexpr uint16_t OutputSwitchError = 1u << 10;
        constexpr uint16_t BreakpointWarning = 1u << 12;
        constexpr uint16_t ValueOutOfRange = 1u << 13;
        constexpr uint16_t ParityError = 1u << 14;
    }
}
