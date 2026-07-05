// PositionControl - demonstrates positioning with the BLSD20Modbus library.
//
// Zeroes the position counter, then repeatedly moves a fixed number of Hall
// counts forward and back, waiting for each move to finish.
//
// Position is counted in Hall sensor switching steps. How many counts equal
// one output revolution depends on your motor's pole count and gear ratio.
//
// Controller factory defaults: ID 1, 115200 baud, 8 data bits, EVEN parity.

#include <BLSD20Modbus.h>

#define RS485_PORT Serial1
constexpr int16_t DE_PIN = -1; // -1 = auto-direction transceiver

ModbusRTU bus(RS485_PORT);
BLSD20Modbus motor(bus, 1);

constexpr int32_t STEP = 20000; // Hall counts per move

void waitUntilStopped()
{
    // Poll the status register until the controller reports a full stop.
    while (motor.isMoving())
    {
        delay(50);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    bus.begin(115200, SERIAL_8E1, DE_PIN);

    if (!motor.ping())
    {
        Serial.print("Controller not found: ");
        Serial.println(motor.lastResultText());
        while (true) {}
    }

    motor.setControlMode(BLSD20ControlMode::Modbus);

    // === Set these for YOUR motor! See the notes in BasicControl. ===
    // Current limit starts at the controller minimum (1000 mA) as a safety
    // net; raise it towards your motor's rated current once tuned.
    motor.setPoleCount(8);        // <-- REQUIRED: number of poles of your motor
    motor.setCurrentLimit(1000);  // <-- mA, raise towards the rated current
    // ================================================================

    motor.setSpeed(800);
    motor.setAcceleration(100);
    motor.setDeceleration(100);
    motor.enableAcceleration(true);
    motor.enableDeceleration(true);

    motor.resetPosition();
    Serial.println("start position zeroed");
}

void loop()
{
    Serial.println("move forward");
    motor.moveOffset(STEP);
    waitUntilStopped();
    Serial.print("position: ");
    Serial.println(motor.getPosition());

    delay(1000);

    Serial.println("move back");
    motor.moveOffset(-STEP);
    waitUntilStopped();
    Serial.print("position: ");
    Serial.println(motor.getPosition());

    delay(1000);
}
