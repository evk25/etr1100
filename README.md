# Case 2: A Reliable Robot Controller (Motors, Sensors, and Power)

**ETR 1100 Innovation Track — Team 2**  
*Evan Kaba, Marko Jovanovic, Logan Holloway, Gianna Mattia*

---

## Overview
This MicroPython firmware and Wokwi circuit ([diagram.json](diagram.json)) adapts the ESP32 robot controller to solve the **Case 2** reliability, predictability, and safety challenges for young learners and instructors:

1. **Predictable Closed-Loop Turns**: Uses the MPU6050 gyroscope (`GyZ`, calibrated at startup) to integrate yaw angle and stop within `±2.0°` of the target angle (exceeding the `±5°` requirement).
2. **Battery Monitoring & Reporting**: Reads simulated 2S LiPo voltage (`6.2V–8.4V`) via ADC GPIO 34 before and during every run. Blocks operation and alerts below `6.6V`.
3. **Instructor E-Stop**: Hardware interrupt on GPIO 26 (`btn_estop`, active LOW) immediately halts motors, lights the red `FAULT LED`, and sounds the buzzer.
4. **Obstacle & Tilt Safety**: Uses the HC-SR04 ultrasonic sensor (stops if `< 15 cm`) and MPU6050 accelerometer (`|AcX|, |AcY| > 12000`) to prevent collisions and tip-overs.
5. **Clear Fault Diagnostics**: OLED + Serial + LED + Buzzer clearly label whether a stop was a **`ROBOT` hardware/safety fault** (`LOW BATT`, `E-STOP`, `TILTED`, `OBSTACLE`, `IMU FAIL`, `NO MOTION`) or a **`PROGRAM` / command issue** (`NO LINE`, `INVALID CMD`).

---

## Hardware Pinout ([diagram.json](diagram.json))
| Component | Pin(s) | Purpose |
| :--- | :--- | :--- |
| **ESP32 DevKit C v4** | — | Main controller (MicroPython v1.22.0) |
| **Left / Right Stepper (A4988)** | GPIO `13/14` (L), `19/18` (R) | `STEP` / `DIR` motor control |
| **MPU6050 IMU** | GPIO `21` (SDA), `22` (SCL) | Yaw gyro (`GyZ`) + tilt detection (`AcX/AcY`) |
| **SSD1306 OLED (128x64)** | GPIO `21` (SDA), `22` (SCL) | Live status, battery, & fault source display |
| **HC-SR04 Ultrasonic** | GPIO `5` (TRIG), `17` (ECHO) | Obstacle stop (`< 15 cm`) |
| **Line Sensor (Switch)** | GPIO `32` | Digital line detection (`1` = line) |
| **Battery Monitor (Pot)** | GPIO `34` (ADC) | 1:3 divider simulation (`6.2V–8.4V`) |
| **START Button** | GPIO `27` | Triggers default `90°` closed-loop turn test |
| **E-STOP Button** | GPIO `26` (Pull-Up, IRQ) | Immediate instructor safety stop |
| **FAULT LED / Buzzer** | GPIO `12` / GPIO `15` | Visual & audio fault indicators |

---

## Battery Reporting Format
Reported on the OLED (`Batt:7.74V 70%`) and over Serial (`BATTERY` / `STATUS` commands and run logs):
```text
BATT:<voltage>V (<percent>%) [<OK|LOW|CRIT>]
```
- `OK`: `>= 6.60V`
- `LOW`: `< 6.60V` (blocks new runs, halts active runs)
- `CRIT`: `< 6.20V` (`0%`)

---

## Accepted Serial Commands
Send newline-terminated commands via the Wokwi Serial Monitor (or UART):

| Command | Description |
| :--- | :--- |
| `HELP` | List all accepted commands |
| `STATUS` | Report battery, IMU, ultrasonic distance, line sensor, and E-Stop state |
| `BATTERY` | Report battery voltage, percentage, and health state |
| `CALIBRATE` | Re-zero MPU6050 `GyZ` bias while stationary |
| `TURN <degrees>` | Closed-loop gyro turn (e.g., `TURN 90`, `TURN -90`) |
| `FORWARD <steps>` | Drive straight for `<steps>` with all safety monitors active |
| `LINE [max_steps]` | Drive forward until line sensor triggers (default `2000` steps) |
| `TEST_TURN [n] [deg]` | Run `n` consecutive turn trials (default `10` x `90°`) and report `±5°` pass rate |
| `STOP` | Trigger immediate software stop |

---

## Testing in Wokwi
1. Open the project in Wokwi (or VS Code Wokwi extension) and start the simulation.
2. Adjust **BATTERY SIM** (`batt_pot`) above ~67% (`>= 6.6V`) so the robot enters `READY`.
3. Press **START** (key `1`) or send `TURN 90` in the Serial Monitor.
   - *Tip*: In Wokwi, click the **MPU6050** component and set `gyroZ` (e.g., `90 °/s`) while turning to simulate physical rotation; if left at `0`, the controller detects `NO MOTION / Wheel/IMU Stall` after 1.8s.
4. Test **E-STOP** (key `e`), **LOW BATT** (turn pot down), **OBSTACLE** (set HC-SR04 `< 15 cm`), and **TILT** (adjust MPU6050 `accelY` in Wokwi).
