#pragma once
#include <Arduino.h>

class MotorDriver {
private:
    uint8_t m_ain1;
    uint8_t m_ain2;
    uint8_t m_pwma;
    uint8_t m_bin1;
    uint8_t m_bin2;
    uint8_t m_pwmb;
    uint8_t m_stby;

    static void setChannel(uint8_t in1, uint8_t in2, uint8_t pwmPin, int16_t speed) {
        if (speed >= 0) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            analogWrite(pwmPin, speed > 255 ? 255 : speed);
        } else {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            int16_t revSpeed = -speed;
            analogWrite(pwmPin, revSpeed > 255 ? 255 : revSpeed);
        }
    }

public:
    MotorDriver(uint8_t ain1, uint8_t ain2, uint8_t pwma,
                uint8_t bin1, uint8_t bin2, uint8_t pwmb,
                uint8_t stby)
        : m_ain1(ain1), m_ain2(ain2), m_pwma(pwma),
          m_bin1(bin1), m_bin2(bin2), m_pwmb(pwmb),
          m_stby(stby) {}

    void init() {
        pinMode(m_ain1, OUTPUT);
        pinMode(m_ain2, OUTPUT);
        pinMode(m_pwma, OUTPUT);
        pinMode(m_bin1, OUTPUT);
        pinMode(m_bin2, OUTPUT);
        pinMode(m_pwmb, OUTPUT);
        pinMode(m_stby, OUTPUT);

        enable();
        stop();
    }

    void enable() {
        digitalWrite(m_stby, HIGH);
    }

    void disable() {
        digitalWrite(m_stby, LOW);
    }

    void setSpeeds(int16_t left, int16_t right) {
        setChannel(m_ain1, m_ain2, m_pwma, left);
        setChannel(m_bin1, m_bin2, m_pwmb, right);
    }

    void stop() {
        setSpeeds(0, 0);
    }
};
