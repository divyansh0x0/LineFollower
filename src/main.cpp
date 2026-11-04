#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "SensorArray.h"
#include "PIDController.h"
#include "Button.h"
#include "CommandParser.h"

// ==========================================
// Robot State & Parameters
// ==========================================
bool running = false;
int16_t baseSpeed = Config::Defaults::BASE_SPEED;
int16_t searchSpeed = Config::Defaults::SEARCH_SPEED;

// ==========================================
// Hardware Abstractions & Modules
// ==========================================
MotorDriver motors(
    Config::Motors::AIN1, Config::Motors::AIN2, Config::Motors::PWMA,
    Config::Motors::BIN1, Config::Motors::BIN2, Config::Motors::PWMB,
    Config::Motors::STBY
);

SensorArray sensors(
    Config::Sensors::S0, Config::Sensors::S1, Config::Sensors::S2,
    Config::Sensors::S3, Config::Sensors::S4
);

PIDController pid(
    Config::Defaults::KP,
    Config::Defaults::KD,
    Config::Defaults::KI
);

ToggleTrigger startTrigger(
    Config::Controls::START_BUTTON,
    Config::Controls::BT_START_PIN,
    Config::Defaults::DEBOUNCE_MS
);

CommandParser commandParser(pid, baseSpeed, searchSpeed, running, Serial);

// ==========================================
// Arduino Setup & Loop
// ==========================================
void setup() {
    Serial.begin(Config::Defaults::SERIAL_BAUD);

    motors.init();
    sensors.init();
    startTrigger.init();

    Serial.println(F("\n=================================="));
    Serial.println(F("   LineFollower Initialized"));
    Serial.println(F("=================================="));
    Serial.println(F("Send 'help' for Bluetooth tuning commands."));
    commandParser.printStatus();
}

void loop() {
    // 1. Process incoming commands from HC-05 BLE / Serial
    commandParser.update();

    // 2. Handle start/stop button or remote hardware trigger
    if (startTrigger.checkTriggered()) {
        running = !running;
        if (running) {
            pid.reset();
            Serial.println(F("[INFO] Robot started via trigger"));
        } else {
            motors.stop();
            Serial.println(F("[INFO] Robot stopped via trigger"));
        }
    }

    // 3. If idle, ensure motors are off and yield
    if (!running) {
        motors.stop();
        return;
    }

    // 4. Sample sensor array
    SensorReading reading = sensors.read();

    // 5. Line loss recovery: execute search rotation if track is lost
    if (reading.isLineLost()) {
        motors.setSpeeds(-searchSpeed, searchSpeed); // Spin left search
        return;
    }

    // 6. Compute PID steering correction from sensor error
    float correction = pid.compute(static_cast<float>(reading.error));

    // 7. Calculate differential motor speeds
    int leftSpeed  = static_cast<int>(baseSpeed - correction);
    int rightSpeed = static_cast<int>(baseSpeed + correction);

    // 8. Constrain motor speeds to allowable range [-255, 255]
    leftSpeed  = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // 9. Actuate motors
    motors.setSpeeds(static_cast<int16_t>(leftSpeed), static_cast<int16_t>(rightSpeed));
}
