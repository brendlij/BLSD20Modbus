// BasicControl - minimal example for the BLSD20Modbus library.
//
// Starts the motor, prints live telemetry for 10 seconds, then stops it.
//
// Works on any Arduino-compatible board that has a spare hardware serial
// port supporting even parity (Teensy, Arduino Mega, ESP32, STM32, SAMD,
// RP2040, ...). SoftwareSerial cannot generate even parity and is not
// supported - the controller ships configured for 8E1.
//
// Controller factory defaults: ID 1, 115200 baud, 8 data bits, EVEN parity.

#include <BLSD20Modbus.h>

// Hardware serial port wired to the RS-485 transceiver.
// Teensy/Mega/STM32/SAMD/RP2040: Serial1.  ESP32: Serial1 or Serial2.
#define RS485_PORT Serial1

// RS-485 direction control:
//   -1  : transceiver switches automatically (e.g. Waveshare isolated
//         converter) - nothing to do.
//   pin : GPIO driving DE + !RE of a plain MAX485-style transceiver.
constexpr int16_t DE_PIN = -1;

ModbusRTU bus(RS485_PORT);
BLSD20Modbus motor(bus, 1); // slave id 1 = factory default

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // 8E1 = 8 data bits, even parity, 1 stop bit - the controller default.
    bus.begin(115200, SERIAL_8E1, DE_PIN);

    if (!motor.ping())
    {
        Serial.print("Controller not found: ");
        Serial.println(motor.lastResultText());
        while (true) {}
    }

    motor.setControlMode(BLSD20ControlMode::Modbus);
    motor.setRotationMode(BLSD20RotationMode::Continuous);

    // === Set these for YOUR motor before running with real hardware! ===
    // Wrong pole count can make the controller run the motor away to full
    // speed (manual section 6.2). The current limit is your safety net: it
    // is kept at the controller minimum (1000 mA) here so an untuned setup
    // stays gentle. Raise it towards your motor's rated current (nameplate)
    // once everything behaves - the motor may barely turn at 1000 mA.
    motor.setPoleCount(8);         // <-- REQUIRED: number of poles of your motor
    motor.setCurrentLimit(1000);   // <-- mA, raise towards the rated current
    // ===================================================================

    motor.setAcceleration(100);
    motor.setDeceleration(100);
    motor.enableAcceleration(true);
    motor.enableDeceleration(true);
    motor.setDirection(BLSD20Direction::Forward);

    motor.setSpeed(500);           // rpm - start slow
    motor.start();
}

void loop()
{
    static uint32_t started = millis();

    Serial.print("rpm=");
    Serial.print(motor.getSpeed());
    Serial.print("  mA=");
    Serial.print(motor.getCurrent());
    Serial.print("  pos=");
    Serial.println(motor.getPosition());

    if (millis() - started > 10000 && motor.isMoving())
    {
        motor.stop();
        Serial.println("stopped.");
    }

    delay(500);
}
