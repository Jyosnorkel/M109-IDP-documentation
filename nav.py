# nav.py - line following, junctions, turns, route actions and the route runner.
from utime import sleep_ms, ticks_ms, ticks_diff, ticks_add
import config as C
import hw
from hw import read, drive, spin, stop, wait_ms

turn_ref = {-1: C.TURN_REF_LEFT_MS, 1: C.TURN_REF_RIGHT_MS}  # fixed calibrated 90 deg times
DIR_NAME = {-1: "left", 1: "right"}
state = {"pole": None, "holding": False}


# ---------------- Line following (milestone 2) ------------------------------------------
class Follower:
    def __init__(self):
        """Set the follower state before line tracking starts."""
        self.last_side = 0
        self.lost_since = None

    def reset(self):
        """Clear the saved line-tracking state."""
        self.lost_since = None

    def step(self, s, k=1.0):
        """Choose the wheel correction from the current line sensor readings."""
        ol, il, ir_, orr = s
        if il and ir_:
            drive(k * C.BASE_SPEED, k * C.BASE_SPEED)
            self.last_side = 0
            self.lost_since = None
        elif ol and il:
            drive(k * C.CORR_SLOW, k * C.CORR_FAST)
            self.last_side = -1
            self.lost_since = None
        elif orr and ir_:
            drive(k * C.CORR_FAST, k * C.CORR_SLOW)
            self.last_side = 1
            self.lost_since = None
        elif ol:
            drive(k * C.CORR_SLOW/1.5, k * C.CORR_FAST)
            self.last_side = -1
            self.lost_since = None
        elif orr:
            drive(k * C.CORR_FAST, k * C.CORR_SLOW/1.5)
            self.last_side = 1
            self.lost_since = None
        elif il:   
            drive(k * C.CORR_SLOW, k * C.CORR_FAST)
            self.last_side = -1
            self.lost_since = None
        elif ir_:
            drive(k * C.CORR_FAST, k * C.CORR_SLOW)
            self.last_side = 1
            self.lost_since = None
        
        else:                           # outer-only white is a recovery cue, not a lost line
            now = ticks_ms()
            outer_only = bool(ol) != bool(orr)
            timeout = C.TURN_RECOVERY_MS if outer_only else C.LOST_TIMEOUT_MS
            if self.lost_since is None:
                self.lost_since = now
            elif ticks_diff(now, self.lost_since) > timeout:
                return False
            if outer_only:
                self.last_side = -1 if ol else 1
            if self.last_side < 0:
                drive(k * C.SEARCH_SLOW, k * C.SEARCH_FAST)
            elif self.last_side > 0:
                drive(k * C.SEARCH_FAST, k * C.SEARCH_SLOW)
            else:
                drive(C.CREEP_SPEED, C.CREEP_SPEED)
        return True


def outers_clear(s):
    """Return whether both outer line sensors are clear."""
    return not s[0] and not s[3]


def all_black(s):
    """Return whether all line sensors read zero."""
    return not any(s)


def middles_on(s):
    """Return whether both middle sensors see the line."""
    return s[1] and s[2]


def recover_after_turn(direction):
    """Follow slowly until centred (middles on, outers clear) before looking for junctions."""
    f = Follower()
    f.last_side = direction
    start = ticks_ms()
    ok = 0
    k = C.CREEP_SPEED / C.BASE_SPEED
    while ticks_diff(ticks_ms(), start) < C.TURN_RECOVERY_MS:
        s = read()
        ok = ok + 1 if middles_on(s) and outers_clear(s) else 0
        if ok >= C.LINE_CONFIRM:
            stop()
            return True
        if not f.step(s, k):
            break
        sleep_ms(C.LOOP_MS)
    stop()
    return False


def identify(first):
    """Creep and OR the outer sensors, so an angled T still reads as BOTH."""
    l, r = first[0], first[3]
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < C.JUNCTION_WINDOW_MS:
        drive(C.CREEP_SPEED, C.CREEP_SPEED)
        s = read()
        l |= s[0]
        r |= s[3]
        sleep_ms(C.LOOP_MS)
    return "BOTH" if (l and r) else ("LEFT" if l else "RIGHT")


def reverse_to_junction(target="LEFT"):
    """Reverse until the selected outer sensor detects a junction."""
    if target not in ("LEFT", "RIGHT", "BOTH"):
        raise ValueError("Unknown reverse target: %s" % target)
    start = ticks_ms()
    event = None
    try:
        while ticks_diff(ticks_ms(), start) < C.RACK_EXIT_BACK_TIMEOUT_MS:
            hw.check_stop()
            s = read()
            on = s[0] if target == "LEFT" else (s[3] if target == "RIGHT" else s[0] and s[3])
            if on:
                event = target
                break
            drive(-C.RACK_EXIT_BACK_SPEED, -C.RACK_EXIT_BACK_SPEED)
            sleep_ms(C.LOOP_MS)
    finally:
        stop()
    return event if event is not None else False



def drive_until(cond, speed=C.CREEP_SPEED, timeout_ms=None):
    """Drive until the condition is met or the timeout expires."""
    n = 0
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < (timeout_ms or C.CLEAR_TIMEOUT_MS):
        drive(speed, speed)
        n = n + 1 if cond(read()) else 0
        if n >= C.CLEAR_CONFIRM:
            return True
        sleep_ms(C.LOOP_MS)
    return False


# ---------------- Turns (milestone 2) ------------------------------------------------------
last_spin = {"early": None}       # ms of the first fresh line seen before the window, last spin


def spin_until_middles_on(t0, earliest, latest, rack_entry=False):
    """Observe the whole spin, but accept only a fresh line after leaving the start."""
    n = off = 0
    departed = False
    early_hit = last_spin["early"] = None
    need = C.RACK_ENTRY_CONFIRM if rack_entry else C.LINE_CONFIRM
    while True:
        hw.check_stop()
        elapsed = ticks_diff(ticks_ms(), t0)
        if elapsed >= latest:
            stop()
            return None
        s = read()
        off = off + 1 if not middles_on(s) else 0
        if off >= C.CLEAR_CONFIRM:
            departed = True
        on = middles_on(s) and (not rack_entry or outers_clear(s))
        n = n + 1 if departed and on else 0
        if n >= need and elapsed < earliest and early_hit is None:
            early_hit = last_spin["early"] = elapsed
        if elapsed < earliest:
            sleep_ms(C.LOOP_MS)
            continue
        if n >= need:
            stop()
            return elapsed
        sleep_ms(C.LOOP_MS)


def find_line():
    """Search for the line after a turn."""
    t0 = ticks_ms()
    for direction, ms in ((-1, C.LINE_SWEEP_MS), (1, 2 * C.LINE_SWEEP_MS), (-1, C.LINE_SWEEP_MS)):
        spin(direction)
        start = ticks_ms()
        n = 0
        while ticks_diff(ticks_ms(), start) < ms:
            hw.check_stop()
            n = n + 1 if middles_on(read()) else 0
            if n >= C.LINE_CONFIRM:
                stop()
                return True
            sleep_ms(C.LOOP_MS)
    stop()
    return False


def follow_for(ms, why="advance", stop_at_end=False, speed=C.CREEP_SPEED):
    """Follow the line for a set time."""
    if ms <= 0:
        return True
    f = Follower()
    k = speed / C.BASE_SPEED
    start = ticks_ms()
    off = 0
    while ticks_diff(ticks_ms(), start) < ms:
        s = read()
        off = off + 1 if not (s[1] or s[2]) else 0
        if stop_at_end and off >= C.CLEAR_CONFIRM:
            stop()
            return "END"
        if not f.step(s, k):
            stop()
            return False
        sleep_ms(C.LOOP_MS)
    return True


def advance_over_junction():
    """Drive straight over the junction."""
    if C.ADVANCE_FOLLOW_LINE:
        return follow_for(C.ADVANCE_MS, "advance")
    drive(C.CREEP_SPEED, C.CREEP_SPEED)
    wait_ms(C.ADVANCE_MS)
    stop()
    return True


def turn(direction, advance=True, rack_entry=False):
    """90 deg turn using a fixed calibrated reference and sensor confirmation window."""
    name = DIR_NAME[direction]
    ref = turn_ref[direction]
    if advance and not advance_over_junction():
        return False
    t0 = ticks_ms()
    spin(direction)
    if rack_entry:
        lo, hi = int(ref * C.RACK_ENTRY_MIN_FACTOR), int(ref * C.RACK_ENTRY_MAX_FACTOR)
    else:
        lo, hi = int(ref * (1 - C.TURN_TOLERANCE)), int(ref * (1 + C.TURN_TOLERANCE))
    elapsed = spin_until_middles_on(t0, lo, hi, rack_entry)
    stop()
    if elapsed is None and last_spin["early"] is not None:
        # It passed a line before the window (robot already angled, or the advance overshot):
        # that was the new line. Spin back to it rather than give up.
        back = hi - last_spin["early"] + C.LINE_SWEEP_MS
        return spin_back_to_line(-direction, back)
    if elapsed is None:
        # No line at all yet (robot angled the other way): keep turning a little further.
        more = int(ref * C.TURN_EXTRA)
        return spin_back_to_line(direction, more)
    return elapsed is not None


def rack_exit_turn(direction):
    """Turn from a rack stub onto the main line."""
    if not advance_over_junction():
        return False
    t0 = ticks_ms()
    try:
        spin(direction)
        elapsed = spin_until_middles_on(t0, 0, C.TURN_TIMEOUT_MS)
    finally:
        stop()
    return elapsed is not None



def spin_back_to_line(direction, ms):
    """Spin direction for up to ms; stop when both middles are on a line. True if found."""
    t0 = ticks_ms()
    spin(direction)
    n = 0
    while ticks_diff(ticks_ms(), t0) < ms:
        hw.check_stop()
        n = n + 1 if middles_on(read()) else 0
        if n >= C.LINE_CONFIRM:
            stop()
            return True
        sleep_ms(C.LOOP_MS)
    stop()
    return False


def half_turn_ms():
    """Return the configured half-turn duration."""
    return C.TURN_180_MS


def timed_180(where):
    """Run the configured timed half-turn."""
    ms = {"bay": C.TURN_180_BAY_MS, "rack": C.TURN_180_RACK_MS}.get(where) or half_turn_ms()
    spin(-1)
    wait_ms(ms)
    stop()


def checked_half_turn():
    """Turn until the line is confirmed."""
    half = C.CHECKED_HALF_TURN_MS or half_turn_ms()
    lo, hi = int(half * (1 - C.TURN_TOLERANCE)), int(half * (1 + C.TURN_TOLERANCE))
    t0 = ticks_ms()
    spin(-1)
    elapsed = spin_until_middles_on(t0, lo, hi)
    stop()
    return elapsed is not None


# ---------------- Actions -------------------------------------------------------------------
def arm_ready():
    """Put the arm in its ready-to-pick-up position."""
    stop()
    hw.grabber.set_angle(C.GRABBER_OPEN)
    hw.lift.set_angle(C.LIFT_BOTTOM)
    wait_ms(C.ARM_READY_SETTLE_MS)
    return True


def pickup():
    """Pick up a package and identify its pole."""
    if C.PICKUP_FORWARD_MS is None:
        if not drive_until(all_black):
            return False
    else:
        drive_until(all_black, timeout_ms=C.PICKUP_FORWARD_MS)
    stop()
    sleep_ms(100)
    hw.grabber.set_angle(C.GRABBER_CLOSED)
    wait_ms(C.GRIP_SETTLE_MS)
    state["holding"] = True
    state["pole"] = hw.read_pole()
    hw.lights.pole(state["pole"])
    hw.lift.set_angle(C.LIFT_CARRY)
    wait_ms(C.MOVE_SETTLE_MS)
    # Holding the package now, so move on to where the timed turn was measured: the far edge
    # of the box (all black), or just PICKUP_AFTER_GRAB_MS if that is set.
    if C.PICKUP_AFTER_GRAB_MS is None:
        drive_until(all_black)
    elif C.PICKUP_AFTER_GRAB_MS > 0:
        drive(C.CREEP_SPEED, C.CREEP_SPEED)
        wait_ms(C.PICKUP_AFTER_GRAB_MS)
    stop()
    sleep_ms(100)
    timed_180("bay")                   # timed: inside the white box the line check can't work
    sleep_ms(100)
    if not drive_until(outers_clear):
        return False
    s = read()
    return bool(s[1] or s[2]) or find_line()   # one middle on: the recovery follow re-centres


def release_at_bay():
    """Lower and release an ignored package at the current bay, without driving."""
    stop()
    hw.lift.set_angle(C.LIFT_BOTTOM)
    wait_ms(C.MOVE_SETTLE_MS)
    hw.grabber.set_angle(C.GRABBER_OPEN)
    wait_ms(C.ARM_READY_SETTLE_MS)
    state["holding"] = False
    hw.lights.pole(None)
    return True


def ramp_straight():
    """Drive straight up the ramp and find the centre line."""
    if C.RAMP_FOLLOW_LINE:
        if not follow_for(C.RAMP_STRAIGHT_MS, "ramp", speed=C.RAMP_SPEED):
            return find_line()
    else:
        drive(C.RAMP_SPEED, C.RAMP_SPEED)
        wait_ms(C.RAMP_STRAIGHT_MS)
    stop()
    s = read()
    return bool(s[1] or s[2]) or find_line()


def scan_slot():
    """Scan a rack slot and return the nearest reading."""
    vals = []

    def look():
        """Wait for the sensor to settle, then read the slot distance."""
        sleep_ms(C.DIST_SETTLE_MS)
        vals.append(hw.dist_bay.read_mm(C.SLOT_SCAN_SAMPLES))

    look()
    n = C.SLOT_SCAN_STEPS
    for d in [-1] * n + [1] * (2 * n) + [-1] * n:      # left, across to the right, back to centre
        drive(d * C.SLOT_SCAN_SPEED, -d * C.SLOT_SCAN_SPEED)
        wait_ms(C.SLOT_SCAN_STEP_MS)
        stop()
        look()
    good = [v for v in vals if v is not None]
    return (min(good) if good else None), vals


def check_slot(deck="lower"):
    """Check whether a rack slot is occupied."""
    cfg = C.SLOT_CHECK[deck]
    if not follow_for(cfg["advance_ms"], "slot check advance"):
        return False
    stop()
    d, vals = scan_slot() if hw.dist_bay else (None, [])
    occupied = d is not None and d < cfg["occupied_mm"]
    if not checked_half_turn() and not find_line():
        return False
    return "OCCUPIED" if occupied else "EMPTY"


def raise_lift():
    """Stop and raise the package to LIFT_RAISED (no settle wait if it is already there)."""
    stop()
    if hw.lift.last_angle != C.LIFT_RAISED:
        hw.lift.set_angle(C.LIFT_RAISED)
        wait_ms(C.MOVE_SETTLE_MS)
    return True


def rack_approach(deck="lower"):
    """Approach the rack slot and raise the lift."""
    if not follow_for(C.RACK_RAISE_AT_MS[deck], "rack approach"):
        return False
    return raise_lift()


def drive_for(direction, ms):
    """Straight at RACK_REVERSE_SPEED for ms, then stop: +1 forward, -1 backward."""
    if ms <= 0:
        return
    drive(direction * C.RACK_REVERSE_SPEED, direction * C.RACK_REVERSE_SPEED)
    wait_ms(ms)
    stop()


def place():
    """Place the package in the rack."""
    stop()
    sleep_ms(100)
    hw.grabber.set_angle(C.GRABBER_OPEN)
    wait_ms(C.MOVE_SETTLE_MS)
    hw.lights.pole(None)
    state["holding"] = False
    drive_for(-1, C.PLACE_BACKOFF_MS)
    hw.grabber.set_angle(C.GRABBER_CLOSED)
    wait_ms(C.ARM_READY_SETTLE_MS)
    drive_for(1, C.PLACE_BACKOFF_MS + C.PLACE_PUSH_MS)
    drive_for(-1, C.RACK_REVERSE_MS + C.PLACE_PUSH_MS)
    sleep_ms(100)
    timed_180("rack")
    sleep_ms(100)
    return True


def hub_turn(direction):
    """At the centre-line crossing along the bottom line: turn to face the ramp, stop."""
    ok = turn(direction)
    stop()
    return ok


def hub_turn_around():
    """At the hub crossing coming down the ramp: half-turn to face the ramp, stop."""
    if not advance_over_junction():
        return False
    return checked_half_turn()


def wait_green():
    """Traffic light: wait until the IR receiver reports green (+10 per top-deck package)."""
    stop()
    if hw.ir is None:
        return True
    start = window = ticks_ms()
    seen = 0
    while ticks_diff(ticks_ms(), start) < C.WAIT_GREEN_TIMEOUT_MS:
        hw.check_stop()
        if ticks_diff(ticks_ms(), window) > C.IR_WINDOW_MS:
            window, seen = ticks_ms(), 0
        if hw.ir_active():
            seen += 1
            if seen >= C.IR_CONFIRM:
                return True
        sleep_ms(1)
    return True


ACTIONS = {
    "STRAIGHT":         lambda: drive_until(outers_clear, C.BASE_SPEED),
    "LEFT":             lambda: turn(-1),
    "RIGHT":            lambda: turn(1),
    "RACK_ENTRY_LEFT":  lambda: turn(-1),
    "RACK_ENTRY_RIGHT": lambda: turn(1),
    "RACK_EXIT_LEFT":   lambda: rack_exit_turn(-1),
    "RACK_EXIT_RIGHT":  lambda: rack_exit_turn(1),
    "LEFT_IN_PLACE":    lambda: turn(-1, advance=False),
    "RIGHT_IN_PLACE":   lambda: turn(1, advance=False),
    "ARM_READY":        arm_ready,
    "RAISE_LIFT":       raise_lift,
    "RAMP_STRAIGHT":    ramp_straight,
    "CHECK_SLOT":       check_slot,
    "CHECK_SLOT_UPPER": lambda: check_slot("upper"),
    "PICKUP":           pickup,
    "RACK_APPROACH":    rack_approach,
    "RACK_APPROACH_UPPER": lambda: rack_approach("upper"),
    "PLACE":            place,
    "WAIT_GREEN":       wait_green,
    "HUB_LEFT":         lambda: hub_turn(-1),
    "HUB_RIGHT":        lambda: hub_turn(1),
    "HUB_TURN_AROUND":  hub_turn_around,
}
RECOVER_AFTER = {"LEFT": -1, "RIGHT": 1, "RACK_ENTRY_LEFT": -1, "RACK_ENTRY_RIGHT": 1,
                 "LEFT_IN_PLACE": -1, "RIGHT_IN_PLACE": 1, "PICKUP": -1, "PLACE": -1}
NO_COOLDOWN = ("LEFT_IN_PLACE", "RIGHT_IN_PLACE", "ARM_READY", "RAISE_LIFT", "WAIT_GREEN",
               "RACK_APPROACH", "RACK_APPROACH_UPPER", "CHECK_SLOT", "CHECK_SLOT_UPPER")


# ---------------- Route runner (milestone 2 main loop) -----------------------------------------
def _route_fail(title, reason, step, route, t0):
    """Log a route failure and stop the robot."""
    stop()
    return False


def run_route(route, title):
    """Run navigation steps and return the result."""
    f = Follower()
    step = hits = line_end = 0
    seen_line = False
    t0 = step_start = ticks_ms()
    # No junctions in the first COOLDOWN_MS: just after a turn-round the robot can drift a sensor
    # onto the line it is on, which read as a junction and turned it inside a rack stub (2 Nov).
    armed_at = ticks_add(t0, C.COOLDOWN_MS)

    while step < len(route):
        if ticks_diff(ticks_ms(), t0) > C.ROUTE_TIMEOUT_MS:
            return _route_fail(title, "route_timeout", step, route, t0)
        want, action, note = route[step][0], route[step][1], route[step][2]
        limit = route[step][3] if len(route[step]) > 3 else None
        waited = ticks_diff(ticks_ms(), step_start)

        s = read()
        event = None
        expected = want[5:] if want.startswith("BACK_") else want
        if want == "NOW":
            event = "NOW"
        elif want.startswith("BACK_"):
            event = reverse_to_junction(expected)
            if event is False:
                return _route_fail(title, "rack_exit_back_timeout", step, route, t0)
            s = read()
        elif want == "TIMEOUT":
            if s[1] or s[2]:
                seen_line, line_end = True, 0
            elif seen_line:
                line_end += 1
            if line_end >= C.CLEAR_CONFIRM:
                event = "LINE_END"
            elif limit is not None and waited >= limit:
                event = "TIMEOUT"
        else:
            hits = hits + 1 if (ticks_diff(ticks_ms(), armed_at) >= 0 and (s[0] or s[3])) else 0
            if hits >= C.EVENT_CONFIRM:
                hits = 0
                event = identify(s)
            elif limit is not None and waited >= limit:
                event = "TIMEOUT"

        if event:
            if expected not in ("ANY", "TIMEOUT", "NOW") and event != expected:
                if want.startswith("BACK_"):
                    return _route_fail(title, "rack_exit_back_wrong_junction", step, route, t0)
            step += 1
            res = ACTIONS[action]()
            if res is False:
                return _route_fail(title, action + "_failed", step - 1, route, t0)
            if isinstance(res, str):    # slot check: turned round in the stub
                if not recover_after_turn(-1):
                    return _route_fail(title, "no_line_after_" + action, step - 1, route, t0)
                return res
            if action in RECOVER_AFTER and not recover_after_turn(RECOVER_AFTER[action]):
                return _route_fail(title, "no_line_after_" + action, step - 1, route, t0)
            f.reset()
            armed_at = ticks_add(ticks_ms(), 0 if action in NO_COOLDOWN else C.COOLDOWN_MS)
            step_start = ticks_ms()
            seen_line, line_end, hits = False, 0, 0
            continue

        if not f.step(s):
            return _route_fail(title, "line_lost", step, route, t0)
        sleep_ms(C.LOOP_MS)

    stop()
    return True
