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
Configured in [Config.h](file:///home/divyansh/Projects/LineFollower/include/Config.h) as pure 16-bit integers (`int16_t`) for zero FPU emulation overhead:

| Parameter | Default Value | Description |
| :--- | :--- | :--- |
| `Kp` | `105` | Proportional gain (responsiveness to error) |
| `Kd` | `25` | Derivative gain (damping oscillations) |
| `baseSpeed` | `40` | Default forward PWM duty cycle (0–255) |
| `searchSpeed`| `80` | Spin recovery PWM duty cycle when line is lost (0–255) |

### Pure Numbers Protocol (HC-05 BLE / UART)
The robot firmware uses no parser classes or string matching. Parameters are sent and echoed back as pure space-separated numbers:

$$\text{Format: } \langle K_p \rangle \quad \langle K_d \rangle \quad \langle \text{baseSpeed} \rangle \quad \langle \text{searchSpeed} \rangle \quad \langle \text{running} \rangle$$

| Input | Example | Behavior | Robot Echo |
| :--- | :--- | :--- | :--- |
| `1` | `1` | Start the robot | `105 25 40 80 1` |
| `0` | `0` | Stop the robot | `105 25 40 80 0` |
| `<kp> <kd>` | `120 30` | Update $K_p$ and $K_d$ | `120 30 40 80 0` |
| `<kp> <kd> <base>` | `120 30 50` | Update $K_p$, $K_d$, and Base Speed | `120 30 50 80 0` |
| `<kp> <kd> <base> <search> <run>` | `120 30 50 80 1` | Update all parameters and run | `120 30 50 80 1` |
| *(empty newline)* | `\n` | Query current parameters | `105 25 40 80 0` |

---

## Project Structure

```text
.
├── include/
│   ├── Button.h          # Non-blocking debounced edge trigger for button and BT pin
│   ├── Config.h          # Hardware pinouts and default configuration constants
│   ├── MotorDriver.h     # Zero-cost TB6612FNG dual motor driver abstraction
│   ├── PIDController.h   # Fast 16-bit integer PD controller
│   ├── SensorArray.h     # 5-channel digital IR sensor reader & error calculator
│   └── README            # PlatformIO include documentation
├── src/
│   └── main.cpp          # Clean, decoupled application entry point (pure numbers)
├── controller.py         # Python controller & live tuner (pure numbers protocol)
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

## Python Controller & Live Tuner

A Python utility ([controller.py](file:///home/divyansh/Projects/LineFollower/controller.py)) is included to tune PID gains, adjust motor speeds, and control the robot live over Bluetooth (or USB Serial).

### 1. Requirements
```bash
pip install pyserial rich
```

### 2. Connect via Bluetooth (HC-05)
On Linux, pair the HC-05 and bind it to an RFCOMM port:
```bash
# Pair and bind (replace with your HC-05 MAC address)
sudo rfcomm bind 0 98:D3:31:XX:XX:XX
# Created device is /dev/rfcomm0
```
*(On Windows or macOS, pair the HC-05 in system Bluetooth settings to obtain a virtual COM port like `COM3` or `/dev/cu.HC-05`).*

### 3. Interactive Live Tuning Session
Launch the interactive tuning shell:
```bash
./controller.py
# Or specify port directly:
./controller.py --port /dev/rfcomm0
```

Inside the interactive shell:
- **`p 120`** / **`d 30`** / **`b 50`**: Set parameters directly.
- **`+p`** / **`-p`**: Increment / decrement $K_p$ by step size.
- **`+d`** / **`-d`**: Increment / decrement $K_d$ by step size.
- **`+b`** / **`-b`**: Increment / decrement Base Speed by step size.
- **`step kp 2.5`**: Change the increment step size.
- **`start`** / **`stop`** / **`toggle`**: Control robot run state.
- **`save fast_track`**: Save active values to `fast_track.json`.
- **`load fast_track`**: Load and apply profile from `fast_track.json`.
- **`status`** / **`?`**: Print formatted parameter status table.
- **`q`** / **`exit`**: Safely stop motors and exit.

### 4. Direct CLI Commands (Single-shot / Scripting)
```bash
# Set parameters and start
./controller.py --port /dev/rfcomm0 --kp 115 --kd 28 --base 45 --start

# Query robot status
./controller.py --port /dev/rfcomm0 --status

# Send emergency stop
./controller.py --port /dev/rfcomm0 --stop

# Apply a saved profile
./controller.py --port /dev/rfcomm0 --load fast_track.json
```

---

## License

This project is open-source and distributed under the [MIT License](file:///home/divyansh/Projects/LineFollower/LICENSE).
