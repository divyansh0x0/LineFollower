#pragma once
#include <Arduino.h>

class ToggleTrigger {
private:
    uint8_t m_buttonPin;
    uint8_t m_btStartPin;
    unsigned long m_debounceMs;
    unsigned long m_lastTriggerTime;
    bool m_lastButtonState;
    bool m_lastBtState;

public:
    ToggleTrigger(uint8_t buttonPin, uint8_t btStartPin, unsigned long debounceMs = 300)
        : m_buttonPin(buttonPin),
          m_btStartPin(btStartPin),
          m_debounceMs(debounceMs),
          m_lastTriggerTime(0),
          m_lastButtonState(HIGH),
          m_lastBtState(LOW) {}

    void init() {
        pinMode(m_buttonPin, INPUT_PULLUP);
        pinMode(m_btStartPin, INPUT);
    }

    bool checkTriggered() {
        unsigned long now = millis();
        bool triggered = false;

        bool currentButtonState = digitalRead(m_buttonPin);
        bool currentBtState = digitalRead(m_btStartPin);

        // Falling edge on active-low button (pressed)
        if (m_lastButtonState == HIGH && currentButtonState == LOW) {
            if (now - m_lastTriggerTime >= m_debounceMs) {
                triggered = true;
                m_lastTriggerTime = now;
            }
        }

        // Rising edge on HC-05 start / state line
        if (m_lastBtState == LOW && currentBtState == HIGH) {
            if (now - m_lastTriggerTime >= m_debounceMs) {
                triggered = true;
                m_lastTriggerTime = now;
            }
        }

        m_lastButtonState = currentButtonState;
        m_lastBtState = currentBtState;

        return triggered;
    }
};
