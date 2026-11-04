# LineFollower Robot

An autonomous high-performance line follower robot built on the Arduino framework (ATmega328P / Arduino Nano) with a 5-sensor infrared reflectance array, TB6612FNG dual motor driver, and a Proportional-Derivative (PD) control loop.

---

## Table of Contents
- [Overview](#overview)
- [Hardware & Pin Configuration](#hardware--pin-configuration)
  - [Microcontroller & Motor Driver](#microcontroller--motor-driver)
  - [Sensor Array](#sensor-array)
  - [Controls & Bluetooth (HC-05)](#controls--bluetooth-hc-05)
- [Control Algorithm](#control-algorithm)
  - [Weighted Error Calculation](#weighted-error-calculation)
  - [PD Controller & Motor Speeds](#pd-controller--motor-speeds)
  - [Line-Loss Recovery](#line-loss-recovery)
- [Tuning Parameters](#tuning-parameters)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Build and Flash](#build-and-flash)
- [License](#license)

---

## Overview

The robot uses a 5-channel digital IR sensor array to track a line (dark on light or light on dark) with closed-loop PD feedback to steer differential drive DC motors. It features:
- **PD-based steering correction** for smooth path tracking and fast corner recovery.
- **Bidirectional motor control** supporting counter-rotation / pivot turns on sharp bends.
- **Line loss recovery behavior** to locate the track when off-course.
- **Toggle Start/Stop switch** with debounce handling.
- **HC-05 Bluetooth module support** for wireless start/stop triggering and telemetry.

---

## Hardware & Pin Configuration

The firmware targets an **Arduino Nano (ATmega328P)** operating at 16 MHz.

### Microcontroller & Motor Driver

The robot uses a **TB6612FNG** (or compatible H-Bridge) dual DC motor driver:

| Driver Pin | Arduino Nano Pin | Description |
| :--- | :--- | :--- |
| **STBY** | `D10` | Standby enable (Active HIGH) |
| **PWMA** | `D5` (PWM) | Left motor speed control |
| **AIN1** | `D11` | Left motor direction 1 |
| **AIN2** | `D12` | Left motor direction 2 |
| **PWMB** | `D6` (PWM) | Right motor speed control |
| **BIN1** | `D9` | Right motor direction 1 |
| **BIN2** | `D8` | Right motor direction 2 |

### Sensor Array

A 5-channel reflectance sensor array (e.g., TCRT5000 / QTR-5A) is wired to analog pins configured in digital input mode:

| Sensor | Nano Pin | Position | Weight |
| :--- | :--- | :--- | :--- |
| **S0** | `A0` | Far Left | `-4` |
| **S1** | `A1` | Left | `-2` |
| **S2** | `A2` | Center | `0` |
| **S3** | `A3` | Right | `+2` |
| **S4** | `A4` | Far Right | `+4` |

*Note: In code, sensors are inverted (`!digitalRead(Sx)`) assuming active-low line detection.*

### Controls & Bluetooth (HC-05)

| Component | Nano Pin | Description |
| :--- | :--- | :--- |
| **Push Button** | `D4` | `INPUT_PULLUP` toggle start/stop switch (300 ms debounce) |
| **HC-05 State / Start** | `D7` | Remote start trigger signal |
| **HC-05 UART** | `TX / RX` | Cross-connected (Nano TX &rarr; HC-05 RX, Nano RX &larr; HC-05 TX) |

> [!TIP]
> Ensure a voltage divider or level shifter is used on the Nano TX &rarr; HC-05 RX line if your HC-05 module requires 3.3V logic levels.

---

## Control Algorithm

### Weighted Error Calculation

Sensor values are evaluated as binary active signals ($s_i \in \{0, 1\}$). The offset error is calculated using weighted positional values:

$$\text{Error} = (-4 \cdot s_0) + (-2 \cdot s_1) + (0 \cdot s_2) + (2 \cdot s_3) + (4 \cdot s_4)$$

- **Error = 0**: Robot is centered directly over the line ($s_2 = 1$).
- **Error < 0**: Robot has drifted right (line is to the left), requires left turn.
- **Error > 0**: Robot has drifted left (line is to the right), requires right turn.

### PD Controller & Motor Speeds

The correction factor is computed using Proportional ($P$) and Derivative ($D$) terms:

$$P = \text{Error}$$

$$D = \text{Error} - \text{LastError}$$

$$\text{Correction} = (K_p \cdot P) + (K_d \cdot D)$$

The motor speeds are modulated symmetrically around the base speed and clamped to $[-255, 255]$:

$$\text{Left Speed} = \text{baseSpeed} - \text{Correction}$$

$$\text{Right Speed} = \text{baseSpeed} + \text{Correction}$$

- If speed $\ge 0$, the motor is driven forward with `analogWrite(PWM, speed)`.
- If speed $< 0$, the motor is driven in reverse with `analogWrite(PWM, -speed)` for tight pivoting.

### Line-Loss Recovery

When all sensors lose track of the line ($\sum_{i=0}^4 s_i = 0$):
- The robot executes an autonomous search rotation:
  $$\text{motor}(-\text{searchSpeed}, +\text{searchSpeed})$$
- By default, it spins left until the line is re-acquired.

---

## Tuning Parameters & Bluetooth Commands

### Default Parameters
Configured in [Config.h](file:///home/divyansh/Projects/LineFollower/include/Config.h):

| Parameter | Default Value | Description |
| :--- | :--- | :--- |
| `Kp` | `105.0` | Proportional gain (responsiveness to instantaneous error) |
| `Kd` | `25.0` | Derivative gain (damping oscillations and overshoot) |
| `Ki` | `0.0` | Integral gain (steady-state offset elimination) |
| `baseSpeed` | `40` | Default forward PWM duty cycle (0–255) |
| `searchSpeed`| `80` | Spin recovery PWM duty cycle when line is lost (0–255) |

### Real-Time HC-05 BLE / Serial Commands
Connect to the HC-05 via any Serial Bluetooth Terminal app (baud: `9600`) to tune parameters live while testing:

| Command | Example | Description |
| :--- | :--- | :--- |
| `p <val>` / `kp <val>` | `p 120` or `p=120` | Set Proportional Gain ($K_p$) |
| `d <val>` / `kd <val>` | `d 30` or `d=30` | Set Derivative Gain ($K_d$) |
| `i <val>` / `ki <val>` | `i 0.5` | Set Integral Gain ($K_i$) |
| `b <val>` / `base <val>` | `b 50` | Set Base Speed (0–255) |
| `search <val>` | `search 90` | Set Line-Lost Search Speed (0–255) |
| `start` / `run` | `start` | Start the robot |
| `stop` / `halt` | `stop` | Stop the robot |
| `toggle` | `toggle` | Toggle run/stop state |
| `status` / `?` | `status` | View all current settings & state |
| `help` / `h` | `help` | Print command cheat sheet |

*(Tip: Sending `p`, `d`, `i`, `b`, or `search` with no value queries the current setting).*

---

## Project Structure

```text
.
├── include/
│   ├── Button.h          # Non-blocking debounced edge trigger for button and BT pin
│   ├── CommandParser.h   # Real-time HC-05 Bluetooth command parser & tuner
│   ├── Config.h          # Hardware pinouts and default configuration constants
│   ├── MotorDriver.h     # Zero-cost TB6612FNG dual motor driver abstraction
│   ├── PIDController.h   # PD/PID controller with dynamic tuning & anti-windup
│   ├── SensorArray.h     # 5-channel digital IR sensor reader & error calculator
│   └── README            # PlatformIO include documentation
├── src/
│   └── main.cpp          # Clean, decoupled application entry point
├── old.cpp               # Original monolithic reference implementation
├── platformio.ini        # PlatformIO environment configuration (nanoatmega328)
└── README.md             # Project documentation
```

---

## Getting Started

### Prerequisites

- [PlatformIO Core (CLI)](https://docs.platformio.org/en/latest/core/index.html) or [PlatformIO IDE for VS Code / CLion](https://platformio.org/).

### Build and Flash

1. **Clone the repository:**
   ```bash
   git clone https://github.com/divyansh0x0/LineFollower.git
   cd LineFollower
   ```

2. **Build the firmware:**
   ```bash
   pio run
   ```

3. **Upload to Arduino Nano:**
   ```bash
   pio run --target upload
   ```

4. **Monitor serial output (if configured):**
   ```bash
   pio device monitor
   ```

---

## License

This project is open-source and distributed under the [MIT License](file:///home/divyansh/Projects/LineFollower/LICENSE).
