"""
Case 2: A Reliable Robot Controller (Motors, Sensors, and Power)
ETR 1100 Innovation Track - Team 2

Implements:
1. Startup self-test & MPU6050 gyro bias calibration
2. Battery voltage & percentage monitoring (pre-run and under load)
3. Instructor hardware-interrupt E-Stop (GPIO 26, active LOW)
4. Closed-loop predictable turning via MPU6050 yaw integration (+/- 2 deg target)
5. Straight-line & line-following drive with ultrasonic obstacle & tilt safety stops
6. Clear fault diagnostics (OLED + LED + Buzzer + Serial) distinguishing robot faults from program issues
7. Serial command interface + physical START button for automated & interactive testing
"""

import sys
from machine import SoftI2C, Pin, ADC
from utime import sleep, ticks_ms, ticks_us, ticks_diff
from src.led import LED
from src.button import Button
from src.buzzer import Buzzer
from src.stepper import Stepper
from src.hcsr04 import HCSR04
import src.oled as oled
import src.mpu6050 as mpu6050

try:
    import uselect
    _spoll = uselect.poll()
    _spoll.register(sys.stdin, uselect.POLLIN)
except Exception:
    _spoll = None


# --- HARDWARE CONFIGURATION ---
my_led = LED(12)                                # Status / Fault LED
start_btn = Button(27)                          # Student Start Button
estop_pin = Pin(26, Pin.IN, Pin.PULL_UP)        # Instructor E-Stop Button (active LOW)
my_buzz = Buzzer(15)

leftWheel = Stepper(13, 14)
rightWheel = Stepper(19, 18)

# Line Sensor (Digital OUT on GPIO 32: 0 = White/No Line, 1 = Black Line)
line_sensor = Pin(32, Pin.IN)
LINE_DETECTED = 1

# Ultrasonic Obstacle Sensor (HC-SR04)
ultrasonic = HCSR04(trigger_pin=5, echo_pin=17)
OBSTACLE_STOP_CM = 15

# Battery Monitor via 1:3 Voltage Divider on ADC GPIO 34 (2S LiPo: 6.2V - 8.4V)
battery_adc = ADC(Pin(34))
battery_adc.atten(ADC.ATTN_11DB)                # Full 0 - 3.3V range
FULL_BATTERY_V = 8.4
CRIT_BATTERY_V = 6.2
LOW_BATTERY_V = 6.6

# I2C Peripherals (OLED + MPU6050 IMU)
i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
oled_screen = oled.I2C(128, 64, i2c)
mpu = None
imu_ok = False
try:
    mpu = mpu6050.accel(i2c)
    imu_ok = True
except Exception as e:
    print("[FAULT] IMU_INIT_FAIL:", e)

# --- CONTROL & SAFETY CONSTANTS ---
GYRO_LSB_PER_DPS = 131.0                        # MPU6050 default +/-250 deg/s scale
TURN_TOLERANCE_DEG = 2.0                        # Target accuracy (well within +/-5 deg spec)
MAX_TURN_STEPS = 1200                           # Safety cap on steps per turn
STALL_CHECK_MS = 1800                           # Detect wheel slip / disconnected IMU
TILT_THRESHOLD = 12000                          # Accelerometer threshold for tip-over
MAX_LINE_STEPS = 2000                           # Safety timeout for line-seeking run

gyro_z_bias = 0.0
emergency_stop = False


def trigger_estop(pin):
    """Hardware interrupt handler: latches emergency stop immediately."""
    global emergency_stop
    emergency_stop = True


estop_pin.irq(trigger=Pin.IRQ_FALLING, handler=trigger_estop)


# --- DIAGNOSTICS & TELEMETRY ---
def read_battery():
    """Returns (voltage_v, percentage, state_str) for a 2S LiPo via 1:3 divider."""
    raw = battery_adc.read()
    pin_v = (raw / 4095.0) * 3.3
    batt_v = round(pin_v * 3.0, 2)
    pct = int(((batt_v - CRIT_BATTERY_V) / (FULL_BATTERY_V - CRIT_BATTERY_V)) * 100)
    pct = max(0, min(100, pct))
    if batt_v < CRIT_BATTERY_V:
        state = "CRIT"
    elif batt_v < LOW_BATTERY_V:
        state = "LOW"
    else:
        state = "OK"
    return batt_v, pct, state


def format_battery_report():
    batt_v, pct, state = read_battery()
    return f"BATT:{batt_v:.2f}V ({pct}%) [{state}]"


def display_status(line1, line2="", line3="", line4=""):
    """Displays up to 4 short lines (<=16 chars each) on the 128x64 OLED."""
    oled_screen.clear()
    oled_screen.text(line1[:16], 0, 0)
    if line2:
        oled_screen.text(line2[:16], 0, 2)
    if line3:
        oled_screen.text(line3[:16], 0, 4)
    if line4:
        oled_screen.text(line4[:16], 0, 5)
    oled_screen.show()


def alert_fault(reason, detail="", source="ROBOT"):
    """Visual, audio, and serial alert distinguishing robot faults from code issues."""
    my_led.on()
    display_status("FAULT: " + reason, detail, "Src: " + source, "Check & Reset")
    print(f"[FAULT][{source}] {reason} | {detail}")
    for _ in range(3):
        my_buzz.beep_once()
        sleep(0.12)


def read_obstacle_cm():
    """Safely reads ultrasonic distance; returns None if out of range or sensor error."""
    try:
        return ultrasonic.distance_cm()
    except OSError:
        return None


def calibrate_gyro(samples=20):
    """Measures stationary MPU6050 Z-axis bias to prevent turn drift."""
    global gyro_z_bias
    if not imu_ok or mpu is None:
        return False
    total = 0.0
    for _ in range(samples):
        vals = mpu.get_values()
        total += vals["GyZ"]
        sleep(0.01)
    gyro_z_bias = total / samples
    print(f"[INFO] Gyro Z calibrated. Bias={gyro_z_bias:.2f} LSB")
    return True


def check_safety_monitors(step_count, check_obstacle=True):
    """
    Checks all safety interlocks. Returns None if safe, or (fault_code, detail) if triggered.
    """
    if emergency_stop or estop_pin.value() == 0:
        return ("ESTOP", "Instructor Stop")

    if step_count % 50 == 0:
        batt_v, pct, state = read_battery()
        if state != "OK":
            return ("LOW_BATT", f"Batt {batt_v}V {pct}%")

    if imu_ok and mpu is not None:
        vals = mpu.get_values()
        if abs(vals["AcY"]) > TILT_THRESHOLD or abs(vals["AcX"]) > TILT_THRESHOLD:
            return ("TILTED", "Robot Tipped")

    if check_obstacle and step_count % 25 == 0:
        dist = read_obstacle_cm()
        if dist is not None and dist < OBSTACLE_STOP_CM:
            return ("OBSTACLE", f"Blocked at {dist}cm")

    return None


# --- MOTION PRIMITIVES ---
def turn_degrees(target_deg):
    """
    Closed-loop turn using MPU6050 GyZ integration.
    Positive degrees = turn right (CW), Negative degrees = turn left (CCW).
    Returns (status_str, measured_deg, steps_taken).
    """
    global emergency_stop
    emergency_stop = False
    my_led.off()

    if not imu_ok or mpu is None:
        alert_fault("IMU FAIL", "Check I2C wiring", source="ROBOT")
        return ("IMU_FAIL", 0.0, 0)

    batt_v, pct, state = read_battery()
    if state != "OK":
        alert_fault("LOW BATT", f"{batt_v}V ({pct}%)", source="ROBOT")
        return ("LOW_BATT", 0.0, 0)

    if estop_pin.value() == 0:
        alert_fault("E-STOP HELD", "Release E-Stop", source="ROBOT")
        return ("ESTOP", 0.0, 0)

    direction_right = target_deg >= 0
    goal = abs(target_deg)
    if goal < TURN_TOLERANCE_DEG:
        return ("REACHED", 0.0, 0)

    my_buzz.beep_once()
    display_status("TURNING...", f"Goal: {target_deg:+.1f}deg", "Closed-Loop IMU", "E-Stop: Pin 26")
    print(f"[RUN] TURN {target_deg:+.1f} deg | {format_battery_report()}")

    angle_accum = 0.0
    steps = 0
    start_ms = ticks_ms()
    last_us = ticks_us()
    status = "TIMEOUT"

    # Left turn: both wheels dir=0; Right turn: both wheels dir=1 (differential spin)
    left_dir = 1 if direction_right else 0
    right_dir = 1 if direction_right else 0

    while steps < MAX_TURN_STEPS:
        fault = check_safety_monitors(steps, check_obstacle=False)
        if fault is not None:
            status, detail = fault
            alert_fault(status, detail, source="ROBOT")
            return (status, round(angle_accum, 2), steps)

        now_us = ticks_us()
        dt = ticks_diff(now_us, last_us) / 1000000.0
        last_us = now_us

        vals = mpu.get_values()
        gyro_dps = (vals["GyZ"] - gyro_z_bias) / GYRO_LSB_PER_DPS
        angle_accum += abs(gyro_dps) * dt

        if angle_accum >= (goal - TURN_TOLERANCE_DEG):
            status = "REACHED"
            break

        # Detect wheel slip / disconnected IMU if no rotation is measured after STALL_CHECK_MS
        if ticks_diff(ticks_ms(), start_ms) > STALL_CHECK_MS and angle_accum < 1.0:
            status = "STALL"
            alert_fault("NO MOTION", "Wheel/IMU Stall", source="ROBOT")
            return (status, round(angle_accum, 2), steps)

        leftWheel.move_one_step(left_dir)
        rightWheel.move_one_step(right_dir)
        steps += 1

    signed_measured = angle_accum if direction_right else -angle_accum
    err = signed_measured - target_deg
    if status == "REACHED":
        my_buzz.beep_once()
        display_status("TURN OK", f"Goal: {target_deg:+.0f}deg", f"Meas: {signed_measured:+.1f}deg", f"Err: {err:+.1f}deg")
        print(f"[DONE] TURN REACHED | target={target_deg:.1f} actual={signed_measured:.2f} err={err:+.2f} steps={steps}")
    else:
        alert_fault("TURN LIMIT", f"Meas: {signed_measured:+.1f}deg", source="ROBOT")

    return (status, round(signed_measured, 2), steps)


def move_forward(max_steps=MAX_LINE_STEPS, stop_on_line=True):
    """
    Drives forward with closed-loop safety monitors (E-Stop, Tilt, Obstacle, Battery)
    and optional line-sensor stop.
    """
    global emergency_stop
    emergency_stop = False
    my_led.off()

    batt_v, pct, state = read_battery()
    if state != "OK":
        alert_fault("LOW BATT", f"{batt_v}V ({pct}%)", source="ROBOT")
        return ("LOW_BATT", 0)

    if estop_pin.value() == 0:
        alert_fault("E-STOP HELD", "Release E-Stop", source="ROBOT")
        return ("ESTOP", 0)

    my_buzz.beep_once()
    mode_lbl = "Tracking Line" if stop_on_line else f"Steps: {max_steps}"
    display_status("RUNNING...", mode_lbl, "E-Stop: Pin 26")
    print(f"[RUN] FORWARD max_steps={max_steps} stop_on_line={stop_on_line} | {format_battery_report()}")

    steps_taken = 0
    status = "COMPLETED"

    while steps_taken < max_steps:
        fault = check_safety_monitors(steps_taken, check_obstacle=True)
        if fault is not None:
            status, detail = fault
            alert_fault(status, detail, source="ROBOT")
            return (status, steps_taken)

        if stop_on_line and line_sensor.value() == LINE_DETECTED:
            status = "REACHED"
            break

        leftWheel.move_one_step(0)
        rightWheel.move_one_step(1)
        steps_taken += 1

    if stop_on_line and status != "REACHED":
        status = "NO_LINE"
        alert_fault("NO LINE", f"Ran {steps_taken} steps", source="PROGRAM")
    else:
        my_buzz.beep_once()
        display_status("SUCCESS!", mode_lbl, f"Steps: {steps_taken}")
        print(f"[DONE] FORWARD {status} | steps={steps_taken}")

    return (status, steps_taken)


def run_turn_validation(trials=10, target_deg=90.0):
    """Runs N consecutive turn tests and reports how many landed within +/- 5 deg."""
    print(f"[TEST] Starting {trials}-trial turn validation (target={target_deg} deg, pass<=5.0 deg)")
    passed = 0
    for i in range(1, trials + 1):
        status, measured, steps = turn_degrees(target_deg)
        err = abs(measured - target_deg)
        ok = (status == "REACHED") and (err <= 5.0)
        if ok:
            passed += 1
        print(f"  Trial {i}/{trials}: status={status} measured={measured:.2f} err={err:.2f} -> {'PASS' if ok else 'FAIL'}")
        if status in ("ESTOP", "LOW_BATT", "TILTED", "IMU_FAIL"):
            print("[TEST] Aborted due to safety/hardware fault.")
            break
        sleep(0.5)
    print(f"[TEST] Result: {passed}/{trials} turns within +/-5 deg")
    display_status("TEST DONE", f"Pass: {passed}/{trials}", f"Target: {target_deg:.0f}deg")


# --- SERIAL COMMAND INTERFACE ---
def print_help():
    print("Accepted Commands:")
    print("  HELP                     - Show this command list")
    print("  STATUS                   - Report battery, sensors, and system state")
    print("  BATTERY                  - Report battery voltage, %, and health")
    print("  CALIBRATE                - Re-zero MPU6050 gyro while stationary")
    print("  TURN <deg>               - Closed-loop gyro turn (e.g. TURN 90, TURN -90)")
    print("  FORWARD <steps>          - Drive straight for N steps with safety checks")
    print("  LINE [max_steps]         - Drive forward until line sensor triggers")
    print("  TEST_TURN [n] [deg]      - Run N-trial turn repeatability test (default 10x90)")
    print("  STOP                     - Trigger immediate safety stop")


def print_status():
    batt_v, pct, state = read_battery()
    dist = read_obstacle_cm()
    dist_str = f"{dist}cm" if dist is not None else "OOR"
    estop_str = "ACTIVE" if (emergency_stop or estop_pin.value() == 0) else "OK"
    print(
        f"STATUS | BATT:{batt_v:.2f}V({pct}%)[{state}] | "
        f"IMU:{'OK' if imu_ok else 'FAIL'} | DIST:{dist_str} | "
        f"LINE:{line_sensor.value()} | ESTOP:{estop_str}"
    )


def handle_command(raw_line):
    parts = raw_line.strip().split()
    if not parts:
        return
    cmd = parts[0].upper()

    if cmd == "HELP":
        print_help()
    elif cmd == "STATUS":
        print_status()
    elif cmd == "BATTERY":
        print(format_battery_report())
    elif cmd == "CALIBRATE":
        ok = calibrate_gyro()
        print("CALIBRATE:", "OK" if ok else "FAIL")
    elif cmd == "TURN":
        if len(parts) < 2:
            print("[FAULT][PROGRAM] INVALID CMD: Usage TURN <degrees>")
            return
        try:
            deg = float(parts[1])
            turn_degrees(deg)
        except ValueError:
            print("[FAULT][PROGRAM] INVALID CMD: degrees must be a number")
    elif cmd == "FORWARD":
        if len(parts) < 2:
            print("[FAULT][PROGRAM] INVALID CMD: Usage FORWARD <steps>")
            return
        try:
            steps = int(parts[1])
            move_forward(max_steps=steps, stop_on_line=False)
        except ValueError:
            print("[FAULT][PROGRAM] INVALID CMD: steps must be an integer")
    elif cmd == "LINE":
        steps = int(parts[1]) if len(parts) > 1 else MAX_LINE_STEPS
        move_forward(max_steps=steps, stop_on_line=True)
    elif cmd == "TEST_TURN":
        n = int(parts[1]) if len(parts) > 1 else 10
        deg = float(parts[2]) if len(parts) > 2 else 90.0
        run_turn_validation(trials=n, target_deg=deg)
    elif cmd == "STOP":
        alert_fault("CMD STOP", "Software Stop", source="PROGRAM")
    else:
        print(f"[FAULT][PROGRAM] UNKNOWN CMD '{cmd}'. Type HELP for commands.")


def poll_serial_command():
    if _spoll is None:
        return None
    if _spoll.poll(0):
        return sys.stdin.readline()
    return None


# --- STARTUP & MAIN LOOP ---
print("=== Case 2: Reliable Robot Controller ===")
calibrate_gyro()
print_status()
print_help()

while True:
    emergency_stop = False
    my_led.off()

    batt_v, pct, state = read_battery()
    if state != "OK":
        alert_fault("LOW BATT", f"{batt_v}V ({pct}%)", source="ROBOT")
        sleep(2)
        continue

    display_status("READY", f"Batt:{batt_v}V {pct}%", "Press START", "or Serial CMD")

    # Wait for either physical START button or a Serial command
    while True:
        if emergency_stop or estop_pin.value() == 0:
            alert_fault("E-STOP HELD", "Release E-Stop", source="ROBOT")
            sleep(1)
            break

        cmd_line = poll_serial_command()
        if cmd_line is not None:
            handle_command(cmd_line)
            sleep(1)
            break

        if start_btn.is_pressed():
            # Default physical-button run: closed-loop 90-degree turn test
            turn_degrees(90.0)
            sleep(1.5)
            break

        sleep(0.02)
