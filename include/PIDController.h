#pragma once
#include <Arduino.h>

class PIDController {
private:
    int16_t m_kp;
    int16_t m_kd;
    int16_t m_lastError;

public:
    explicit PIDController(int16_t kp = 105, int16_t kd = 25)
        : m_kp(kp), m_kd(kd), m_lastError(0) {}

    int16_t compute(int16_t error) {
        int16_t p = m_kp * error;
        int16_t d = m_kd * (error - m_lastError);
        m_lastError = error;
        return p + d;
    }

    void reset() {
        m_lastError = 0;
    }

    [[nodiscard]] int16_t getKp() const { return m_kp; }
    [[nodiscard]] int16_t getKd() const { return m_kd; }

    void setKp(int16_t kp) { m_kp = kp; }
    void setKd(int16_t kd) { m_kd = kd; }

    void setGains(int16_t kp, int16_t kd) {
        m_kp = kp;
        m_kd = kd;
    }
};
