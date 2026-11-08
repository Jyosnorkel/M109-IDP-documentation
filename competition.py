# competition.py - IDP M1 2026 competition run (main program).
#
# Files on the Pico: competition.py, config.py, hw.py, nav.py, graphs.py
#   main.py on the Pico (see main.py)     run at power-up, waits for the start button
#
# Algorithm
#   1. Amber off, arm open and down, hall sensor calibrated. Wait for the start button.
#   2. Survey rack B lower; serve right loading areas 3 and 4.
#   3. Survey rack A lower; serve left loading areas 2 and 1.
#        Route directly from the current node to each next task.
#        South -> lower rack. Release north packages at their bay and skip them.
#        No upper-rack survey or ramp trips. No new pickup after 3:30.
#   4. Return to the hub and stop. A route failure stops in place.
import config as C
import hw
import nav
import graphs as G
from utime import ticks_ms, ticks_diff


class BayPlan:
    """Attempt each configured bay once per competition run, in configured order."""

    def __init__(self):
        """Create an empty record of visited bays."""
        self.visited = set()

    def peek(self, side=None):
        """Return the next unvisited bay without marking it visited."""
        for bay_side, far in C.BAY_ORDER:
            if (side is None or bay_side == side) and (bay_side, far) not in self.visited:
                return bay_side, far
        return None

    def next(self, side=None):
        """Mark and return the next unvisited bay."""
        bay = self.peek(side)
        if bay is not None:
            self.visited.add(bay)
        return bay


class SlotPlan:
    """Per area (rack, deck), one flag per slot: used[...][i] once we put a package in slot
    i + 1, occupied[...][i] if the survey found it full. A package goes to the first slot that
    is neither, i.e. the empty slot the survey found (slots after it are unchecked)."""

    def __init__(self):
        """Create empty used and occupied flags for the rack slots."""
        self.used = {(rack, deck): [False] * C.SLOTS_PER_RACK
                     for rack in ("A", "B") for deck in ("lower", "upper")}
        self.occupied = {area: [False] * C.SLOTS_PER_RACK for area in self.used}

    def next(self, rack, deck):
        """Return the first slot that is neither used nor occupied."""
        used, occ = self.used[(rack, deck)], self.occupied[(rack, deck)]
        for i in range(len(used)):
            if not used[i] and not occ[i]:
                return i + 1
        return None

    def mark_used(self, rack, deck, slot):
        """Mark a slot as used after delivery."""
        self.used[(rack, deck)][slot - 1] = True

    def mark_occupied(self, rack, deck, slot):
        """Mark a slot as occupied after a survey check."""
        self.occupied[(rack, deck)][slot - 1] = True


def survey(slots, areas=None, position=None, return_to_hub=True):
    """Survey rack areas until an empty slot is found."""
    found = {}
    if position is None:
        position = {"node": "hub"}
    if areas is None:
        areas = C.SURVEY_ORDER
    current = position["node"]
    last = None
    for side, deck in areas:
        rack = "B" if side == "R" else "A"
        found[(rack, deck)] = None
        for slot in range(1, C.SLOTS_PER_RACK + 1):
            label = "%s%d%s" % (rack, slot, "L" if deck == "lower" else "U")
            route = G.survey_to(current, side, deck, slot)
            res = nav.run_route(route, "check " + label)
            if res not in ("EMPTY", "OCCUPIED"):
                position["node"] = None
                return False
            current = G.rack_node(side, deck, slot)
            position["node"] = current
            last = side, deck, slot, label
            if res == "EMPTY":
                found[(rack, deck)] = slot
                break
            slots.mark_occupied(rack, deck, slot)
    if last is not None and return_to_hub:
        side, deck, slot, label = last
        if nav.run_route(G.track.to_route(current, "hub"), label + " -> hub") is not True:
            position["node"] = None
            return False
        position["node"] = "hub"
    return found


def deliver(side, far, slots):
    """Deliver a held package to the next available rack slot."""
    pole = nav.state["pole"]
    if pole not in ("N", "S"):
        return None
    rack = "B" if side == "R" else "A"
    deck = "lower"
    slot = slots.next(rack, deck)
    if slot is None:
        return None
    label = "%s%d%s" % (rack, slot, "L" if deck == "lower" else "U")
    entry = G.track.to_route(G.BAY_NAMES[(side, far)], G.rack_node(side, deck, slot),
                             holding=True)
    if nav.run_route(entry + G.slot_visit(deck), "deliver to " + label) is not True:
        return None
    slots.mark_used(rack, deck, slot)
    return label, deck, slot


def mission():
    # Start from a known state and calibrate the hall sensor with the gripper closed.
    """Run the rack survey, pickups, deliveries, and return to the hub."""
    hw.stop()
    hw.lights.off()
    hw.lights.pole(None)
    hw.lift.set_angle(C.LIFT_BOTTOM)
    hw.grabber.set_angle(C.GRABBER_CLOSED)       # calibrate in the pose the magnet is read in
    hw.wait_ms(C.ARM_READY_SETTLE_MS)
    hw.calibrate_hall()
    hw.grabber.set_angle(C.GRABBER_OPEN)
    hw.wait_press()
    hw.arm_stop_button()
    hw.lights.flash()
    # Start the run clock after the start button is pressed.
    t0 = ticks_ms()

    # Track which bays and rack slots are used, and where the robot is on the course.
    slots = SlotPlan()
    bays = BayPlan()
    delivered = trips = 0
    max_trips = C.MAX_TRIPS

    position = {"node": "hub"}
    # Slot checks need the distance sensor; skip the survey if it is disabled or unavailable.
    survey_enabled = C.SURVEY and hw.dist_bay is not None

    # Zero-trip mode: survey only.
    if max_trips == 0 and survey_enabled:
        lower_areas = tuple((side, deck) for side, deck in C.SURVEY_ORDER if deck == "lower")
        survey(slots, areas=lower_areas, position=position)

    finished = max_trips == 0
    # Work through the right side and rack B, then the left side and rack A.
    for side, rack in (("R", "B"), ("L", "A")):
        if finished or (max_trips is not None and trips >= max_trips):
            break
        if bays.peek(side) is None:
            continue
        if ticks_diff(ticks_ms(), t0) > C.LAST_TRIP_START_MS:
            break

        if survey_enabled:
            # Find the first empty lower slot before collecting packages on this side.
            found = survey(slots, areas=((side, "lower"),),
                           position=position, return_to_hub=False)
            if found is False:
                break

        while bays.peek(side) is not None:
            # Stop before starting another pickup if the trip limit or time cutoff is reached.
            if max_trips is not None and trips >= max_trips:
                finished = True
                break
            if ticks_diff(ticks_ms(), t0) > C.LAST_TRIP_START_MS:
                finished = True
                break
            _, far = bays.next(side)
            trips += 1
            bay = G.BAY_NAMES[(side, far)]
            # Head to the next bay from the last known position, ready for pickup.
            pickup_route = [("NOW", "ARM_READY", "gripper open, lift down")]
            pickup_route += G.track.to_route(position["node"], bay)
            res = nav.run_route(pickup_route, position["node"] + " -> " + bay)
            if res is not True or not nav.state["holding"]:
                # A failed route or pickup leaves the robot's position uncertain.
                position["node"] = None
                finished = True
                break
            position["node"] = bay
            # Deliver to the next available slot and save the rack position for the next route.
            got = deliver(side, far, slots)
            if not got:
                position["node"] = None
                finished = True
                break
            label, deck, slot = got
            position["node"] = G.rack_node(side, deck, slot)
            delivered += 1

    # Known completed nodes can be used for a final parking route. A failed route
    # invalidates position, so do not guess a path after losing the robot.
    if position["node"] not in (None, "hub"):
        current = position["node"]
        if nav.run_route(G.track.to_route(current, "hub"), current + " -> hub") is not True:
            position["node"] = None
        else:
            position["node"] = "hub"

    # Leave the motors and status lights off when the mission ends.
    hw.stop()
    hw.lights.pole(None)
    hw.lights.off()


def run():
    """Run the mission and leave the hardware stopped when it ends."""
    try:
        mission()
    except Exception:
        pass
    finally:
        hw.stop()
        hw.lights.pole(None)
        hw.lights.off()


if __name__ == "__main__":
    run()
