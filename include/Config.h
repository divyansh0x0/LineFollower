#pragma once
#include <Arduino.h>

namespace Config {
    // TB6612FNG Dual H-Bridge Motor Driver Pins
    namespace Motors {
        constexpr uint8_t AIN1 = 11;
        constexpr uint8_t AIN2 = 12;
        constexpr uint8_t PWMA = 5;

        constexpr uint8_t BIN1 = 9;
        constexpr uint8_t BIN2 = 8;
        constexpr uint8_t PWMB = 6;

        constexpr uint8_t STBY = 10;
    }

    // 5-Channel Reflectance / IR Sensors
    namespace Sensors {
        constexpr uint8_t S0 = A0; // Far Left
        constexpr uint8_t S1 = A1; // Left
        constexpr uint8_t S2 = A2; // Center
        constexpr uint8_t S3 = A3; // Right
        constexpr uint8_t S4 = A4; // Far Right
        constexpr uint8_t COUNT = 5;
    }

    // Controls & Remote Triggers
    namespace Controls {
        constexpr uint8_t START_BUTTON = 4; // Onboard push button (INPUT_PULLUP)
        constexpr uint8_t BT_START_PIN = 7; // HC-05 Start / State trigger pin
    }

    // Default Control & Tuning Parameters
    namespace Defaults {
        constexpr float KP = 105.0f;
        constexpr float KD = 25.0f;
        constexpr float KI = 0.0f;

        constexpr int16_t BASE_SPEED = 40;
        constexpr int16_t SEARCH_SPEED = 80;
        constexpr int16_t MAX_SPEED = 255;

        constexpr unsigned long SERIAL_BAUD = 9600;
        constexpr unsigned long DEBOUNCE_MS = 300;
    }
}
