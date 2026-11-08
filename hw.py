# hw.py - hardware: motors, servos, LEDs, line sensors, hall, bay distance sensor, button, IR.
from machine import Pin, PWM, ADC, I2C, Timer
from utime import sleep_ms, ticks_ms, ticks_diff, ticks_add
import config as C

# ---------------- Motors ------------------------------------------------------------
class Motor:
    """TB6612FNG channel, course test_motor.py convention: dir 0 = forward, 1 kHz PWM."""

    def __init__(self, dir_pin, pwm_pin, reversed=False):
        self.dir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.reversed = reversed
        self.off()

    def set(self, speed):
        speed = max(-100, min(100, speed))
        forward = (speed >= 0) != self.reversed
        self.dir.value(0 if forward else 1)
        self.pwm.duty_u16(int(65535 * abs(speed) / 100))

    def off(self):
        self.pwm.duty_u16(0)


left = Motor(C.LEFT_DIR, C.LEFT_PWM, C.LEFT_REVERSED)
right = Motor(C.RIGHT_DIR, C.RIGHT_PWM, C.RIGHT_REVERSED)


# ---------------- Stop button ---------------------------------------------------------
class StopRequested(Exception):
    pass


button = Pin(C.START_BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)
_stop = {"requested": False, "armed_at": None}


def _stop_irq(pin):
    a = _stop["armed_at"]
    if a is None or ticks_diff(ticks_ms(), a) < 0:
        return
    if pin.value() == C.BUTTON_PRESSED:
        _stop["requested"] = True
        left.off()
        right.off()


def pressed():
    return button.value() == C.BUTTON_PRESSED


def wait_press():
    """Wait for one press and release of the start button."""
    while not pressed():
        sleep_ms(10)
    sleep_ms(30)
    while pressed():
        sleep_ms(10)
    sleep_ms(200)


def arm_stop_button():
    """From now on a press stops the run."""
    _stop["requested"] = False
    _stop["armed_at"] = ticks_add(ticks_ms(), 500)
    edge = Pin.IRQ_RISING if C.BUTTON_PRESSED == 1 else Pin.IRQ_FALLING   # the press edge
    button.irq(trigger=edge, handler=_stop_irq)


def check_stop():
    if _stop["requested"]:
        stop()
        raise StopRequested()


def stop():
    left.off()
    right.off()


def drive(l_speed, r_speed):
    check_stop()
    if l_speed > 0 and r_speed > 0:    # trim forward driving only (milestone 2); spins untouched
        l_speed *= C.LEFT_TRIM
        r_speed *= C.RIGHT_TRIM
    left.set(l_speed)
    right.set(r_speed)


def spin(direction):
    """direction -1 = turn left on the spot, +1 = turn right."""
    drive(direction * C.SPIN_SPEED, -direction * C.SPIN_SPEED)


def wait_ms(ms):
    """Sleep that still notices the stop button."""
    end = ticks_add(ticks_ms(), ms)
    while ticks_diff(end, ticks_ms()) > 0:
        check_stop()
        sleep_ms(C.LOOP_MS)


# ---------------- Line sensors ----------------------------------------------------------
sensors = [Pin(p, Pin.IN, Pin.PULL_DOWN) for p in C.SENSOR_PINS]


def read():
    """(OL, IL, IR, OR) with 1 = over the white line."""
    return tuple(1 if p.value() == C.LINE_LEVEL else 0 for p in sensors)


# ---------------- Servos ------------------------------------------------------------------
class Servo:
    """270 deg servo, course test_servo.py mapping: 0.5 ms = 0 deg, 2.5 ms = 270 deg, 50 Hz.
    Moves SERVO_STEP_DEG every SERVO_STEP_MS (milestone 2)."""
    MIN_DUTY = 1638
    MAX_DUTY = 8192

    def __init__(self, pin_no):
        self.servo = PWM(Pin(pin_no))
        self.servo.freq(50)
        self.last_angle = None

    def _command(self, angle):
        self.servo.duty_u16(int(self.MIN_DUTY + angle / 270 * (self.MAX_DUTY - self.MIN_DUTY)))
        self.last_angle = angle

    def set_angle(self, angle):
        angle = max(0, min(270, angle))
        if self.last_angle is None:     # unknown start position: go straight there
            self._command(angle)
            return
        while self.last_angle != angle:
            remaining = angle - self.last_angle
            step = min(C.SERVO_STEP_DEG, abs(remaining))
            sleep_ms(C.SERVO_STEP_MS)
            self._command(self.last_angle + (step if remaining > 0 else -step))


grabber = Servo(C.GRABBER_PIN)
lift = Servo(C.LIFT_PIN)


# ---------------- LEDs ----------------------------------------------------------------------
class Lights:
    """Amber: off in the start box, flashing while running. Red = north, green = south."""

    def __init__(self):
        self.amber = Pin(C.LED_AMBER_PIN, Pin.OUT)
        self.red = Pin(C.LED_RED_PIN, Pin.OUT)
        self.green = Pin(C.LED_GREEN_PIN, Pin.OUT)
        self.timer = Timer()
        self.flashing = False
        self.off()
        self.pole(None)

    def _toggle(self, _t):
        self.amber.value(1 - self.amber.value())

    def flash(self):
        if not self.flashing:
            self.timer.init(period=C.FLASH_MS, mode=Timer.PERIODIC, callback=self._toggle)
            self.flashing = True

    def off(self):
        self.timer.deinit()
        self.flashing = False
        self.amber.value(1 - C.LED_ON)

    def pole(self, p):
        """"N" -> steady red, "S" -> steady green, None -> both off."""
        self.red.value(C.LED_ON if p == "N" else 1 - C.LED_ON)
        self.green.value(C.LED_ON if p == "S" else 1 - C.LED_ON)


lights = Lights()


# ---------------- Hall sensor -----------------------------------------------------------------
hall = ADC(Pin(C.HALL_PIN))
_hall = {"base": None}


def hall_avg(n=C.HALL_SAMPLES):
    total = 0
    for _ in range(n):
        total += hall.read_u16()
        sleep_ms(2)
    return total // n


def calibrate_hall():
    """No-magnet reading. Call with no package near the sensor, gripper in its grab position."""
    _hall["base"] = hall_avg(64)
    return _hall["base"]


def read_pole():
    """"N" or "S" for the package in the gripper (UNKNOWN_POLE if the reading is unclear)."""
    if _hall["base"] is None:
        calibrate_hall()
    v = hall_avg()
    d = v - _hall["base"]
    if abs(d) < C.HALL_THRESHOLD:
        return C.UNKNOWN_POLE
    return "N" if (d > 0) == C.NORTH_IS_HIGH else "S"


# ---------------- Bay distance sensor -----------------------------------------------------------
class Distance:
    """VL53L0X on its own I2C bus, continuous mode. Needs libs/VL53L0X/VL53L0X.py."""

    def __init__(self, bus, sda, scl):
        from libs.VL53L0X.VL53L0X import VL53L0X
        self.tof = VL53L0X(I2C(bus, sda=Pin(sda), scl=Pin(scl)))
        self.tof.start()

    def read_mm(self, n=C.DIST_SAMPLES):
        """Mean of n valid readings in mm, or None if nothing is in range."""
        vals = []
        for _ in range(n * 2):
            v = self.tof.read()
            if 0 < v <= C.DIST_MAX_VALID_MM:
                vals.append(v)
                if len(vals) >= n:
                    break
        return sum(vals) // len(vals) if vals else None


def _make_distance(cfg):
    if cfg is None:
        return None
    try:
        return Distance(cfg[0], cfg[1], cfg[2])
    except Exception:
        return None


dist_bay = _make_distance(C.DIST_BAY)


# ---------------- IR receiver ------------------------------------------------------------------
ir = Pin(C.IR_PIN, Pin.IN, Pin.PULL_UP) if C.IR_PIN is not None else None


def ir_active():
    return ir is not None and ir.value() == C.IR_ACTIVE
