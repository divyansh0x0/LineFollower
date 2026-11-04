#pragma once
#include <Arduino.h>

class PIDController {
private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_lastError;
    float m_integral;
    float m_integralMax;

public:
    explicit PIDController(float kp = 105.0f, float kd = 25.0f, float ki = 0.0f, float integralMax = 255.0f)
        : m_kp(kp), m_kd(kd), m_ki(ki), m_lastError(0.0f), m_integral(0.0f), m_integralMax(integralMax) {}

    float compute(float error) {
        float p = m_kp * error;

        // Anti-windup clamped integral accumulation
        if (m_ki > 0.0f) {
            m_integral += error;
            if (m_integral > m_integralMax) m_integral = m_integralMax;
            else if (m_integral < -m_integralMax) m_integral = -m_integralMax;
        } else {
            m_integral = 0.0f;
        }
        float i = m_ki * m_integral;

        float d = m_kd * (error - m_lastError);
        m_lastError = error;

        return p + i + d;
    }

    void reset() {
        m_lastError = 0.0f;
        m_integral = 0.0f;
    }

    [[nodiscard]] float getKp() const { return m_kp; }
    [[nodiscard]] float getKd() const { return m_kd; }
    [[nodiscard]] float getKi() const { return m_ki; }

    void setKp(float kp) { m_kp = kp; }
    void setKd(float kd) { m_kd = kd; }
    void setKi(float ki) { m_ki = ki; }

    void setGains(float kp, float kd, float ki = 0.0f) {
        m_kp = kp;
        m_kd = kd;
        m_ki = ki;
    }
};
