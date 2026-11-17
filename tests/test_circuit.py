import builtins
import json
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
DIAGRAM_PATH = ROOT / "diagram.json"
MAIN_PATH = ROOT / "main.py"

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


@pytest.fixture
def diagram():
    with DIAGRAM_PATH.open() as f:
        return json.load(f)


_STATE = {
    "adc_raw": 3200,
    "estop_val": 1,
    "line_val": 0,
    "pulse_us": 2000,
    "imu": {"AcX": 0, "AcY": 0, "AcZ": 16384, "GyZ": 13100},
    "time_us": 0,
    "oled_lines": [],
    "led_on": False,
    "beeps": 0,
}


def _install_micropython_mocks():
    if "machine" in sys.modules:
        return

    builtins.const = lambda x: x

    utime = types.ModuleType("utime")
    utime.sleep = lambda s: None
    utime.sleep_us = lambda s: None

    def ticks_us():
        _STATE["time_us"] += 10000  # 10 ms per call
        return _STATE["time_us"]

    def ticks_ms():
        _STATE["time_us"] += 10000
        return _STATE["time_us"] // 1000

    utime.ticks_us = ticks_us
    utime.ticks_ms = ticks_ms
    utime.ticks_diff = lambda a, b: a - b
    sys.modules["utime"] = utime
    sys.modules["time"] = utime

    machine = types.ModuleType("machine")

    class Pin:
        IN = 0
        OUT = 1
        PULL_UP = 2
        IRQ_FALLING = 3

        def __init__(self, pin, mode=0, pull=None):
            self.pin = pin
            self._val = 1 if pull == Pin.PULL_UP else 0

        def value(self, v=None):
            if v is not None:
                self._val = v
                if self.pin == 12:
                    _STATE["led_on"] = bool(v)
                return v
            if self.pin == 26:
                return _STATE["estop_val"]
            if self.pin == 32:
                return _STATE["line_val"]
            return self._val

        def irq(self, **kwargs):
            pass

    class ADC:
        ATTN_11DB = 0

        def __init__(self, pin):
            pass

        def atten(self, a):
            pass

        def read(self):
            return _STATE["adc_raw"]

    class SoftI2C:
        def __init__(self, **kwargs):
            pass

    class PWM:
        def __init__(self, pin):
            pass

        def freq(self, f):
            pass

        def duty(self, d):
            if d > 0:
                _STATE["beeps"] += 1

    machine.Pin = Pin
    machine.ADC = ADC
    machine.SoftI2C = SoftI2C
    machine.PWM = PWM
    machine.time_pulse_us = lambda *args: _STATE["pulse_us"]
    sys.modules["machine"] = machine

    framebuf = types.ModuleType("framebuf")

    class FrameBuffer1:
        def __init__(self, *args):
            pass

        def fill(self, *args):
            pass

        def text(self, s, x, y, col=1):
            _STATE["oled_lines"].append(s)

    framebuf.FrameBuffer1 = FrameBuffer1
    sys.modules["framebuf"] = framebuf

    import src.oled as _oled
    _oled.OLED.init_display = lambda self: None
    _oled.I2C.write_framebuf = lambda self: None
    _oled.I2C.write_cmd = lambda self, c: None

    import src.mpu6050 as _mpu
    _mpu.accel.__init__ = lambda self, i2c, addr=0x68: None
    _mpu.accel.get_values = lambda self: dict(_STATE["imu"])


@pytest.fixture
def firmware_env():
    """Loads main.py functions with mocked MicroPython hardware peripherals."""
    _install_micropython_mocks()
    _STATE.update({
        "adc_raw": 3200,          # ~7.74V (OK)
        "estop_val": 1,           # 1 = released, 0 = pressed (active LOW)
        "line_val": 0,            # 0 = white, 1 = black line
        "pulse_us": 2000,         # ~34 cm (clear)
        "imu": {"AcX": 0, "AcY": 0, "AcZ": 16384, "GyZ": 13100},  # 100 deg/s
        "time_us": 0,
        "oled_lines": [],
        "led_on": False,
        "beeps": 0,
    })

    src_code = MAIN_PATH.read_text().split("# --- STARTUP & MAIN LOOP ---")[0]
    ns = {}
    exec(src_code, ns)
    ns["gyro_z_bias"] = 0.0
    return ns, _STATE


# --- CIRCUIT / DIAGRAM.JSON TESTS ---

def test_diagram_has_all_case2_parts(diagram):
    part_ids = {p["id"]: p["type"] for p in diagram["parts"]}
    expected = {
        "esp": "board-esp32-devkit-c-v4",
        "led1": "wokwi-led",
        "btn1": "wokwi-pushbutton",
        "btn_estop": "wokwi-pushbutton",
        "line_sw": "wokwi-slide-switch",
        "batt_pot": "wokwi-potentiometer",
        "bz1": "wokwi-buzzer",
        "drv1": "wokwi-a4988",
        "drv2": "wokwi-a4988",
        "stepper1": "wokwi-stepper-motor",
        "stepper2": "wokwi-stepper-motor",
        "ultrasonic1": "wokwi-hc-sr04",
        "oled1": "board-ssd1306",
        "imu1": "wokwi-mpu6050",
    }
    for pid, ptype in expected.items():
        assert pid in part_ids, f"Missing part {pid} in diagram.json"
        assert part_ids[pid] == ptype


def test_diagram_connections_reference_valid_parts(diagram):
    valid_ids = {p["id"] for p in diagram["parts"]} | {"$serialMonitor"}
    for src, dst, *_ in diagram["connections"]:
        src_id = src.split(":")[0]
        dst_id = dst.split(":")[0]
        assert src_id in valid_ids, f"Invalid connection source: {src}"
        assert dst_id in valid_ids, f"Invalid connection target: {dst}"


def test_diagram_esp32_pin_mapping_matches_firmware(diagram):
    conns = {(a, b) for a, b, *_ in diagram["connections"]}
    conns |= {(b, a) for a, b in list(conns)}

    assert ("esp:13", "drv1:STEP") in conns
    assert ("esp:14", "drv1:DIR") in conns
    assert ("esp:19", "drv2:STEP") in conns
    assert ("esp:18", "drv2:DIR") in conns
    assert ("esp:26", "btn_estop:2.r") in conns
    assert ("esp:34", "batt_pot:SIG") in conns
    assert ("esp:32", "line_sw:2") in conns
    assert ("esp:5", "ultrasonic1:TRIG") in conns
    assert ("esp:17", "ultrasonic1:ECHO") in conns
    assert ("esp:21", "oled1:SDA") in conns
    assert ("esp:22", "oled1:SCL") in conns
    assert ("esp:21", "imu1:SDA") in conns
    assert ("esp:22", "imu1:SCL") in conns


# --- FIRMWARE & SAFETY TESTS ---

def test_battery_reporting_ok_low_crit(firmware_env):
    ns, state = firmware_env

    state["adc_raw"] = 3200  # ~7.74V
    v, pct, status = ns["read_battery"]()
    assert status == "OK" and 60 <= pct <= 80
    assert "BATT:" in ns["format_battery_report"]()

    state["adc_raw"] = 2500  # ~6.04V -> CRIT
    v, pct, status = ns["read_battery"]()
    assert status == "CRIT" and pct == 0

    state["adc_raw"] = 2650  # ~6.41V -> LOW
    v, pct, status = ns["read_battery"]()
    assert status == "LOW"


@pytest.mark.parametrize("target", [90.0, -90.0])
def test_closed_loop_turn_within_5_degrees(firmware_env, target):
    ns, state = firmware_env
    status, measured, steps = ns["turn_degrees"](target)
    assert status == "REACHED"
    assert abs(measured - target) <= 5.0
    assert steps > 0


def test_turn_stall_detected_when_imu_shows_no_motion(firmware_env):
    ns, state = firmware_env
    state["imu"]["GyZ"] = 0
    status, measured, _ = ns["turn_degrees"](90.0)
    assert status == "STALL"
    assert state["led_on"] is True


def test_estop_interrupt_halts_motion(firmware_env):
    ns, state = firmware_env
    state["estop_val"] = 0
    status, _ = ns["move_forward"](max_steps=50, stop_on_line=False)
    assert status == "ESTOP"
    assert state["led_on"] is True


def test_obstacle_detection_halts_forward_motion(firmware_env):
    ns, state = firmware_env
    state["pulse_us"] = 500  # ~8 cm (< 15 cm threshold)
    status, _ = ns["move_forward"](max_steps=100, stop_on_line=False)
    assert status == "OBSTACLE"
    assert state["led_on"] is True


def test_tilt_detection_halts_motion(firmware_env):
    ns, state = firmware_env
    state["imu"]["AcY"] = 15000
    status, _ = ns["move_forward"](max_steps=100, stop_on_line=False)
    assert status == "TILTED"
    assert state["led_on"] is True


def test_line_sensor_stops_forward_run(firmware_env):
    ns, state = firmware_env
    state["line_val"] = 1
    status, steps = ns["move_forward"](max_steps=100, stop_on_line=True)
    assert status == "REACHED"
    assert steps == 0


def test_serial_commands_status_and_help(firmware_env, capsys):
    ns, _ = firmware_env
    ns["handle_command"]("HELP")
    ns["handle_command"]("STATUS")
    ns["handle_command"]("BATTERY")
    out = capsys.readouterr().out
    assert "Accepted Commands:" in out
    assert "STATUS | BATT:" in out
    assert "BATT:" in out
