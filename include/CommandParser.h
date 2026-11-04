#pragma once
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "PIDController.h"

class CommandParser {
private:
    PIDController& m_pid;
    int16_t& m_baseSpeed;
    int16_t& m_searchSpeed;
    bool& m_running;
    Stream& m_stream;

    char m_buffer[32];
    uint8_t m_idx;

    static const char* skipPrefix(const char* str, const char* prefix) {
        while (*prefix) {
            if (tolower(static_cast<unsigned char>(*str)) != tolower(static_cast<unsigned char>(*prefix))) {
                return nullptr;
            }
            str++;
            prefix++;
        }
        // Skip whitespace, '=', or ':' separator
        while (*str == ' ' || *str == '=' || *str == ':') {
            str++;
        }
        return str;
    }

    static bool matchExact(const char* str, const char* cmd) {
        while (*cmd) {
            if (tolower(static_cast<unsigned char>(*str)) != tolower(static_cast<unsigned char>(*cmd))) {
                return false;
            }
            str++;
            cmd++;
        }
        return (*str == '\0');
    }

    void handleLine(const char* rawLine) {
        // Skip leading whitespace
        while (*rawLine == ' ') rawLine++;
        if (*rawLine == '\0') return;

        // Exact commands
        if (matchExact(rawLine, "help") || matchExact(rawLine, "h")) {
            printHelp();
            return;
        }

        if (matchExact(rawLine, "status") || matchExact(rawLine, "?")) {
            printStatus();
            return;
        }

        if (matchExact(rawLine, "start") || matchExact(rawLine, "run") || matchExact(rawLine, "go")) {
            m_running = true;
            m_pid.reset();
            m_stream.println(F("[OK] Robot started"));
            return;
        }

        if (matchExact(rawLine, "stop") || matchExact(rawLine, "halt")) {
            m_running = false;
            m_stream.println(F("[OK] Robot stopped"));
            return;
        }

        if (matchExact(rawLine, "toggle")) {
            m_running = !m_running;
            if (m_running) m_pid.reset();
            m_stream.print(F("[OK] Running: "));
            m_stream.println(m_running ? F("YES") : F("NO"));
            return;
        }

        // Parameterized commands (longer prefixes checked first)
        const char* val = nullptr;

        // Kp
        if ((val = skipPrefix(rawLine, "kp")) != nullptr || (val = skipPrefix(rawLine, "p")) != nullptr) {
            if (*val == '\0') {
                m_stream.print(F("[INFO] Kp = "));
                m_stream.println(m_pid.getKp(), 2);
            } else {
                float newKp = static_cast<float>(atof(val));
                m_pid.setKp(newKp);
                m_stream.print(F("[OK] Kp = "));
                m_stream.println(m_pid.getKp(), 2);
            }
            return;
        }

        // Kd
        if ((val = skipPrefix(rawLine, "kd")) != nullptr || (val = skipPrefix(rawLine, "d")) != nullptr) {
            if (*val == '\0') {
                m_stream.print(F("[INFO] Kd = "));
                m_stream.println(m_pid.getKd(), 2);
            } else {
                float newKd = static_cast<float>(atof(val));
                m_pid.setKd(newKd);
                m_stream.print(F("[OK] Kd = "));
                m_stream.println(m_pid.getKd(), 2);
            }
            return;
        }

        // Ki
        if ((val = skipPrefix(rawLine, "ki")) != nullptr || (val = skipPrefix(rawLine, "i")) != nullptr) {
            if (*val == '\0') {
                m_stream.print(F("[INFO] Ki = "));
                m_stream.println(m_pid.getKi(), 2);
            } else {
                float newKi = static_cast<float>(atof(val));
                m_pid.setKi(newKi);
                m_stream.print(F("[OK] Ki = "));
                m_stream.println(m_pid.getKi(), 2);
            }
            return;
        }

        // Base Speed
        if ((val = skipPrefix(rawLine, "base")) != nullptr || (val = skipPrefix(rawLine, "speed")) != nullptr || (val = skipPrefix(rawLine, "b")) != nullptr) {
            if (*val == '\0') {
                m_stream.print(F("[INFO] Base Speed = "));
                m_stream.println(m_baseSpeed);
            } else {
                int newBase = atoi(val);
                m_baseSpeed = constrain(newBase, 0, 255);
                m_stream.print(F("[OK] Base Speed = "));
                m_stream.println(m_baseSpeed);
            }
            return;
        }

        // Search Speed
        if ((val = skipPrefix(rawLine, "search")) != nullptr) {
            if (*val == '\0') {
                m_stream.print(F("[INFO] Search Speed = "));
                m_stream.println(m_searchSpeed);
            } else {
                int newSearch = atoi(val);
                m_searchSpeed = constrain(newSearch, 0, 255);
                m_stream.print(F("[OK] Search Speed = "));
                m_stream.println(m_searchSpeed);
            }
            return;
        }

        // Unrecognized command
        m_stream.print(F("[ERROR] Unknown command: '"));
        m_stream.print(rawLine);
        m_stream.println(F("'. Send 'help' for available commands."));
    }

public:
    CommandParser(PIDController& pid, int16_t& baseSpeed, int16_t& searchSpeed, bool& running, Stream& stream = Serial)
        : m_pid(pid),
          m_baseSpeed(baseSpeed),
          m_searchSpeed(searchSpeed),
          m_running(running),
          m_stream(stream),
          m_idx(0) {
        m_buffer[0] = '\0';
    }

    void update() {
        while (m_stream.available() > 0) {
            char c = static_cast<char>(m_stream.read());
            if (c == '\r' || c == '\n') {
                if (m_idx > 0) {
                    m_buffer[m_idx] = '\0';
                    handleLine(m_buffer);
                    m_idx = 0;
                    m_buffer[0] = '\0';
                }
            } else if (m_idx < sizeof(m_buffer) - 1) {
                m_buffer[m_idx++] = c;
            }
        }
    }

    void printStatus() {
        m_stream.println(F("--- Robot Status ---"));
        m_stream.print(F("State        : "));
        m_stream.println(m_running ? F("RUNNING") : F("STOPPED"));
        m_stream.print(F("Kp           : "));
        m_stream.println(m_pid.getKp(), 2);
        m_stream.print(F("Kd           : "));
        m_stream.println(m_pid.getKd(), 2);
        m_stream.print(F("Ki           : "));
        m_stream.println(m_pid.getKi(), 2);
        m_stream.print(F("Base Speed   : "));
        m_stream.println(m_baseSpeed);
        m_stream.print(F("Search Speed : "));
        m_stream.println(m_searchSpeed);
        m_stream.println(F("--------------------"));
    }

    void printHelp() {
        m_stream.println(F("=== HC-05 BLE Command Guide ==="));
        m_stream.println(F("p <val>      : Set Kp (e.g., 'p 120' or 'p=120')"));
        m_stream.println(F("d <val> : Set Kd (e.g., 'd 30' or 'd=30')"));
        m_stream.println(F("i <val>      : Set Ki (e.g., 'i 0.5')"));
        m_stream.println(F("b <val>      : Set Base Speed (0-255)"));
        m_stream.println(F("search <val> : Set Line-Loss Search Speed (0-255)"));
        m_stream.println(F("start / stop : Start or stop robot"));
        m_stream.println(F("toggle       : Toggle running state"));
        m_stream.println(F("status / ?   : Show current parameters"));
        m_stream.println(F("help / h     : Show this menu"));
        m_stream.println(F("(Tip: Send 'p', 'd', 'b' alone to view current value)"));
        m_stream.println(F("==============================="));
    }
};
