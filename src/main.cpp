#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "SensorArray.h"
#include "PIDController.h"
#include "Button.h"

// ==========================================
// Robot State & Parameters (Pure Numbers)
// ==========================================
bool running = false;
int16_t kp = Config::Defaults::KP;
int16_t kd = Config::Defaults::KD;
int16_t baseSpeed = Config::Defaults::BASE_SPEED;
int16_t searchSpeed = Config::Defaults::SEARCH_SPEED;

// ==========================================
// Hardware Abstractions
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

PIDController pid(Config::Defaults::KP, Config::Defaults::KD);

ToggleTrigger startTrigger(
    Config::Controls::START_BUTTON,
    Config::Controls::BT_START_PIN,
    Config::Defaults::DEBOUNCE_MS
);

// Print current parameters as space-separated numbers:
// <kp> <kd> <baseSpeed> <searchSpeed> <running>
void printValues() {
    Serial.print(kp);
    Serial.print(' ');
    Serial.print(kd);
    Serial.print(' ');
    Serial.print(baseSpeed);
    Serial.print(' ');
    Serial.print(searchSpeed);
    Serial.print(' ');
    Serial.println(running ? 1 : 0);
}

// Fast non-blocking numeric serial reader:
// - Send '0' to stop, '1' to start
// - Send 'kp kd' (e.g. '120 30')
// - Send 'kp kd base' (e.g. '120 30 50')
// - Send 'kp kd base search run' (e.g. '120 30 50 80 1')
// - Send empty newline to query values
void readSerial() {
    static int16_t values[5];
    static uint8_t count = 0;
    static int16_t current = 0;
    static bool inNumber = false;
    static bool negative = false;

    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c >= '0' && c <= '9') {
            current = (current * 10) + (c - '0');
            inNumber = true;
        } else if (c == '-') {
            negative = true;
        } else if (c == ' ' || c == ',' || c == '\t') {
            if (inNumber) {
                if (count < 5) values[count++] = negative ? -current : current;
                current = 0;
                negative = false;
                inNumber = false;
            }
        } else if (c == '\n' || c == '\r') {
            if (inNumber) {
                if (count < 5) values[count++] = negative ? -current : current;
                current = 0;
                negative = false;
                inNumber = false;
            }

            if (count > 0) {
                if (count == 1) {
                    if (values[0] == 0) {
                        running = false;
                        motors.stop();
                    } else if (values[0] == 1) {
                        running = true;
                        pid.reset();
                    } else {
                        kp = values[0];
                    }
                } else {
                    if (count >= 2) {
                        kp = values[0];
                        kd = values[1];
                    }
                    if (count >= 3) baseSpeed = values[2];
                    if (count >= 4) searchSpeed = values[3];
                    if (count >= 5) {
                        running = (values[4] != 0);
                        if (running) pid.reset();
                        else motors.stop();
                    }
                }
                pid.setGains(kp, kd);
                count = 0;
            }

            printValues();
        }
    }
}

// ==========================================
// Arduino Setup & Loop
// ==========================================
void setup() {
    Serial.begin(Config::Defaults::SERIAL_BAUD);

    motors.init();
    sensors.init();
    startTrigger.init();

    printValues();
}

void loop() {
    // 1. Read serial numbers from HC-05 / Serial
    readSerial();

    // 2. Hardware button or BT pin trigger
    if (startTrigger.checkTriggered()) {
        running = !running;
        if (running) {
            pid.reset();
        } else {
            motors.stop();
        }
        printValues();
    }

    // 3. If idle, ensure motors are off and return
    if (!running) {
        motors.stop();
        return;
    }

    // 4. Sample 5-sensor array
    SensorReading reading = sensors.read();

    // 5. Line loss recovery spin
    if (reading.isLineLost()) {
        motors.setSpeeds(-searchSpeed, searchSpeed);
        return;
    }

    // 6. Fast integer PD correction
    int16_t correction = pid.compute(reading.error);

    // 7. Calculate differential motor speeds
    int16_t leftSpeed  = baseSpeed - correction;
    int16_t rightSpeed = baseSpeed + correction;

    // 8. Constrain motor speeds to [-255, 255]
    leftSpeed  = constrain(leftSpeed, -Config::Defaults::MAX_SPEED, Config::Defaults::MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -Config::Defaults::MAX_SPEED, Config::Defaults::MAX_SPEED);

    // 9. Actuate motors
    motors.setSpeeds(leftSpeed, rightSpeed);
}
