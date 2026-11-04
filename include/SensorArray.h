#pragma once
#include <Arduino.h>

struct SensorReading {
    uint8_t raw[5];      // Digital readings (1 = line detected, 0 = background)
    uint8_t activeCount; // Number of sensors detecting the line
    int16_t error;       // Weighted position error

    [[nodiscard]] bool isLineLost() const {
        return activeCount == 0;
    }
};

class SensorArray {
private:
    uint8_t m_pins[5];

public:
    SensorArray(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4)
        : m_pins{s0, s1, s2, s3, s4} {}

    void init() {
        for (uint8_t i = 0; i < 5; ++i) {
            pinMode(m_pins[i], INPUT);
        }
    }

    [[nodiscard]] SensorReading read() const {
        SensorReading reading{};
        // Active low sensors: line is detected when pin reads LOW (!digitalRead)
        for (uint8_t i = 0; i < 5; ++i) {
            reading.raw[i] = !digitalRead(m_pins[i]);
            reading.activeCount += reading.raw[i];
        }

        // Weighted error: (-4 * s0) + (-2 * s1) + (0 * s2) + (2 * s3) + (4 * s4)
        reading.error = (-4 * reading.raw[0]) +
                        (-2 * reading.raw[1]) +
                        ( 0 * reading.raw[2]) +
                        ( 2 * reading.raw[3]) +
                        ( 4 * reading.raw[4]);

        return reading;
    }
};
