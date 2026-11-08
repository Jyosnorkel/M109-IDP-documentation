# Motor pins and direction.
LEFT_DIR, LEFT_PWM = 4, 5
RIGHT_DIR, RIGHT_PWM = 7, 6
LEFT_REVERSED = False
RIGHT_REVERSED = True

# Line sensor pins and line signal.
SENSOR_PINS = (18, 19, 20, 21)
LINE_LEVEL = 1

# Arm servo outputs.
GRABBER_PIN = 15
LIFT_PIN = 13

# Start button and hall sensor inputs.
START_BUTTON_PIN = 22
BUTTON_PRESSED = 0
HALL_PIN = 26

# Status LEDs and their active signal level.
LED_AMBER_PIN = 28
LED_RED_PIN = 27
LED_GREEN_PIN = 17
LED_ON = 1

# Front distance sensor for rack slot checks; set to None to disable.
DIST_BAY = (1, 10, 11)

# Traffic light receiver and its active signal level.
IR_PIN = 16
IR_ACTIVE = 0

# Speeds for following, corrections, searching, and turns.
BASE_SPEED = 100
CORR_FAST = 100

CORR_SLOW = 25
SEARCH_FAST = 100
SEARCH_SLOW = -10
CREEP_SPEED = 100
SPIN_SPEED = 55

# Wheel balance for straight driving.
LEFT_TRIM = 0.93

RIGHT_TRIM = 1.0

# Turn timings and line detection.
TURN_TOLERANCE = 0.2
TURN_REF_LEFT_MS = 1100
TURN_REF_RIGHT_MS = 1000
RACK_ENTRY_MIN_FACTOR = 0.95
RACK_ENTRY_MAX_FACTOR = 1.15
RACK_ENTRY_CONFIRM = 4
LINE_CONFIRM = 3

TURN_MIN_MS = 1100
TURN_TIMEOUT_MS = 5000
TURN_REF_MIN_MS = 1250
TURN_REF_MAX_MS = 4000
TURN_180_MS = 3000
TURN_180_BAY_MS = 2500
TURN_180_RACK_MS = 3000
TURN_EXTRA = 0.4

CHECKED_HALF_TURN_MS = None

# Navigation and sensor timings.
LOOP_MS = 5
EVENT_CONFIRM = 3
JUNCTION_WINDOW_MS = 50
COOLDOWN_MS = 350

# Junction approach and recovery.
ADVANCE_MS = 400

ADVANCE_FOLLOW_LINE = False
CLEAR_CONFIRM = 3
CLEAR_TIMEOUT_MS = 3000
TURN_RECOVERY_MS = 1500

# Rack exit recovery.
RACK_EXIT_BACKTRACK = True
RACK_EXIT_BACK_SPEED = 30
RACK_EXIT_BACK_TIMEOUT_MS = 2000

# Ramp driving.
RAMP_STRAIGHT_MS = 4500
RAMP_SPEED = 100

RAMP_FOLLOW_LINE = False

# Line recovery, pickup, and placement timings.
LINE_SWEEP_MS = 330

RACK_REVERSE_SPEED = 100
RACK_REVERSE_MS = 450
PICKUP_FORWARD_MS = 500

PICKUP_AFTER_GRAB_MS = None

PLACE_BACKOFF_MS = 450

PLACE_PUSH_MS = 10

# Route and rack time limits.
LOST_TIMEOUT_MS = 800
ROUTE_TIMEOUT_MS = 90 * 1000
RACK_TIMEOUT_MS = 2830

# Servo positions and movement settling times.
GRABBER_OPEN = 190
GRABBER_CLOSED = 165
LIFT_BOTTOM = 25
LIFT_CARRY = 70
LIFT_RAISED = 150

SERVO_STEP_DEG = 5
SERVO_STEP_MS = 100
MOVE_SETTLE_MS = 2000
ARM_READY_SETTLE_MS = 500
GRIP_SETTLE_MS = 2000

# Hall sensor readings and package polarity.
HALL_SAMPLES = 32
HALL_THRESHOLD = 850
NORTH_IS_HIGH = False
UNKNOWN_POLE = "S"

# How far to travel up each rack stub before raising the lift.
RACK_RAISE_AT_MS = {"lower": 1600, "upper": 1200}

# Rack slot survey settings and occupied-slot thresholds.
SURVEY = True
SURVEY_ORDER = (("R", "lower"), ("L", "lower"))
SLOT_CHECK = {

    "lower": {"advance_ms": 1520, "occupied_mm": 134},

    "upper": {"advance_ms": 1200, "occupied_mm": 130},
}

# Distance readings and slot scanning.
DIST_SAMPLES = 10
SLOT_SCAN_STEPS = 2
SLOT_SCAN_STEP_MS = 70
SLOT_SCAN_SPEED = 60
SLOT_SCAN_SAMPLES = 3
DIST_MAX_VALID_MM = 2000
DIST_SETTLE_MS = 150

# Traffic light detection timing.
WAIT_GREEN_TIMEOUT_MS = 20000
IR_WINDOW_MS = 200
IR_CONFIRM = 20

# Amber light flash interval.
FLASH_MS = 250

# Mission order and time limits.
LAST_TRIP_START_MS = 3 * 60 * 1000 + 30 * 1000
MAX_TRIPS = None
BAY_ORDER = (("R", False), ("R", True), ("L", False), ("L", True))

# Rack slots and upper-deck junction signal.
SLOTS_PER_RACK = 6

DECK_EVENT = "BOTH"
