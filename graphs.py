"""Track map, shortest paths, and navigation actions for the competition.

Nodes use the competition labels: hub, loading area 1..4, A1L..A6L,
B1L..B6L, A1U..A6U, and B1U..B6U. Junctions add " junction".
Rack/bay departure assumes its check/pickup/place action already turned round.
"""
import config as C


BAY_NAMES = {("R", False): "loading area 3", ("R", True): "loading area 4",
             ("L", False): "loading area 2", ("L", True): "loading area 1"}


def rack_node(side, deck, slot):
    """Build a rack label from its side, deck, and slot number."""
    if side not in ("L", "R") or deck not in ("lower", "upper"):
        raise ValueError("Unknown rack area")
    if not 1 <= slot <= C.SLOTS_PER_RACK:
        raise ValueError("Unknown rack slot: %s" % slot)
    return "%s%d%s" % ("A" if side == "L" else "B", slot,
                       "L" if deck == "lower" else "U")


class TrackGraph:
    DIRECTIONS = ("up", "right", "down", "left")
    OPPOSITE = {"up": "down", "right": "left", "down": "up", "left": "right"}

    def __init__(self):
        """Build the track graph and mark its rack nodes."""
        self.graph = {}
        self.racks = {}
        self.upper_junctions = set()
        self.rack_junctions = set()
        self.link("hub", "left corner", "left")
        self.link("hub", "right corner", "right")
        for side, corner, branch in (("L", "left corner", "right"),
                                     ("R", "right corner", "left")):
            bays = [BAY_NAMES[(side, far)] for far in (False, True)]
            lower = [rack_node(side, "lower", i) for i in range(1, C.SLOTS_PER_RACK + 1)]
            self.chain([corner] + [b + " junction" for b in bays]
                       + [r + " junction" for r in lower], "up")
            for bay in bays:
                self.link(bay + " junction", bay, branch)
            for i, rack in enumerate(lower):
                self.racks[rack] = (side, "lower", i + 1)
                self.rack_junctions.add(rack + " junction")
                self.link(rack + " junction", rack, branch)
        upper = ["deck stub %d" % i for i in range(1, C.SLOTS_PER_RACK + 1)]
        self.chain(["hub"] + upper, "up")
        for i, junction in enumerate(upper):
            self.upper_junctions.add(junction)
            self.rack_junctions.add(junction)
            for side, branch in (("L", "left"), ("R", "right")):
                rack = rack_node(side, "upper", i + 1)
                self.racks[rack] = (side, "upper", i + 1)
                self.link(junction, rack, branch)

    @property
    def start_exits(self):
        """Return the exits from the hub by direction."""
        return {direction: node for node, direction in self.graph["hub"].items()}

    def link(self, a, b, direction):
        """Connect two nodes in the track graph."""
        if direction not in self.OPPOSITE:
            raise ValueError("Unknown direction: %s" % direction)
        self.graph.setdefault(a, {})[b] = direction
        self.graph.setdefault(b, {})[a] = self.OPPOSITE[direction]

    def chain(self, nodes, direction):
        """Connect each neighbouring pair of nodes in one direction."""
        for a, b in zip(nodes, nodes[1:]):
            self.link(a, b, direction)

    @classmethod
    def get_turn(cls, heading, direction):
        """Return the turn needed to face an exit."""
        if heading not in cls.DIRECTIONS or direction not in cls.DIRECTIONS:
            raise ValueError("Unknown heading or direction")
        difference = (cls.DIRECTIONS.index(direction) - cls.DIRECTIONS.index(heading)) % 4
        return ("STRAIGHT", "RIGHT", "U_TURN", "LEFT")[difference]

    def find_route(self, start, destination):
        """Shortest path by link count; the map has no measured edge lengths."""
        for node in (start, destination):
            if node not in self.graph:
                raise ValueError("Unknown node: %s" % node)
        queue, previous, index = [start], {start: None}, 0
        while index < len(queue):
            node = queue[index]
            index += 1
            if node == destination:
                path = []
                while node is not None:
                    path.append(node)
                    node = previous[node]
                path.reverse()
                return path
            for neighbour in self.graph[node]:
                if neighbour not in previous:
                    previous[neighbour] = node
                    queue.append(neighbour)
        raise ValueError("No route from %s to %s" % (start, destination))

    def plan(self, start, destination, heading="up"):
        """Plan a path and turns from the start to the destination."""
        path = self.find_route(start, destination)
        self.get_turn(heading, heading)
        steps = []
        for a, b in zip(path, path[1:]):
            direction = self.graph[a][b]
            steps.append({"from": a, "to": b, "direction": direction,
                          "turn": self.get_turn(heading, direction)})
            heading = direction
        return {"path": path, "steps": steps, "heading": heading}

    def expected_event(self, node, heading):
        """Return the line event expected at this node and heading."""
        if node in self.upper_junctions:
            return C.DECK_EVENT
        turns = {self.get_turn(heading, direction) for direction in self.graph[node].values()}
        left, right = "LEFT" in turns, "RIGHT" in turns
        if left and right:
            return "BOTH"
        if left:
            return "LEFT"
        if right:
            return "RIGHT"
        raise ValueError("No detectable side branch at %s" % node)

    def to_route(self, start, destination, holding=False):
        """Convert a track path into navigation steps."""
        path = self.find_route(start, destination)
        if len(path) == 1:
            raise ValueError("Already at %s" % destination)
        if start != "hub" and start not in self.racks and start not in BAY_NAMES.values():
            raise ValueError("Departure must be from hub, a rack, or a loading bay")
        if destination != "hub" and destination not in self.racks and destination not in BAY_NAMES.values():
            raise ValueError("Destination must be hub, a rack, or a loading bay")
        route = []
        if start == "hub":
            action = self.get_turn("up", self.graph[start][path[1]])
            if action != "STRAIGHT":
                route.append(("NOW", action + "_IN_PLACE", "hub: towards " + path[1]))

        for index in range(1, len(path)):
            previous, node = path[index - 1], path[index]
            heading = self.graph[previous][node]
            if previous == "hub":
                if heading == "up":
                    if holding:
                        route.append(("NOW", "RAISE_LIFT", "raise before the ramp"))
                    route += [("NOW", "WAIT_GREEN", "traffic light before the ramp"),
                              ("BOTH", "STRAIGHT", "start-box top edge"),
                              ("NOW", "RAMP_STRAIGHT", "up the ramp, ignoring reflective tape")]
                else:
                    route.append(("BOTH", "STRAIGHT", "exit start-box edge"))
            if node == "hub":
                route.append(("BOTH", "STRAIGHT", "enter start-box edge"))
            if index == len(path) - 1:
                if node == "hub":
                    action = {"left": "HUB_RIGHT", "right": "HUB_LEFT",
                              "down": "HUB_TURN_AROUND"}[heading]
                    route.append((self.expected_event(node, heading), action,
                                  "hub: face the ramp and stop"))
                elif node in BAY_NAMES.values():
                    route.append(("BOTH", "PICKUP", node + ": grab and turn round"))
                break

            following = path[index + 1]
            action = self.get_turn(heading, self.graph[node][following])
            if action == "U_TURN":
                raise ValueError("Unexpected U-turn at %s" % node)
            if (C.RACK_EXIT_BACKTRACK and start in self.racks and index == 1
                    and following in self.rack_junctions and action in ("LEFT", "RIGHT")):
                action = "RACK_EXIT_" + action
            if following == destination and destination in self.racks and action != "STRAIGHT":
                action = "RACK_ENTRY_" + action
            event = self.expected_event(node, heading)
            if (C.RACK_EXIT_BACKTRACK and start in self.racks and index == 2
                    and node in self.rack_junctions):
                # This node follows a turn out of a rack onto the main line.
                # Recover it instead of skipping it or driving towards the next one.
                if event == "BOTH":
                    # On the upper cross-line, monitor the outer on the exit
                    # turn's side. Only one outer is needed to recover the node.
                    event = self.get_turn(self.graph[path[0]][path[1]],
                                          self.graph[path[1]][path[2]])
                event = "BACK_" + event
            route.append((event, action,
                          "at %s: towards %s" % (node, following)))
        return route


track = TrackGraph()


def mirror(route):
    """Swap left and right actions in a route."""
    def swap(text):
        """Swap left and right words in a route action."""
        return text.replace("LEFT", "#").replace("RIGHT", "LEFT").replace("#", "RIGHT")
    return [(swap(s[0]), swap(s[1])) + tuple(s[2:]) for s in route]


def for_side(route, side):
    """Mirror a route for the left side of the track."""
    return route if side == "R" else mirror(route)


def hub_to_bay(far):
    """Build a route from the hub to a right-side bay."""
    return [("NOW", "ARM_READY", "gripper open, lift down")] + track.to_route("hub", BAY_NAMES[("R", far)])


def bay_to_hub(far):
    """Build a route from a right-side bay back to the hub."""
    return track.to_route(BAY_NAMES[("R", far)], "hub")


def bay_to_lower_entry(far, slot):
    """Build a route from a right-side bay to a lower rack slot."""
    return track.to_route(BAY_NAMES[("R", far)], rack_node("R", "lower", slot))


def hub_to_lower_entry(slot):
    """Build a route from the hub to a lower rack slot."""
    return track.to_route("hub", rack_node("R", "lower", slot))


def lower_exit(slot):
    """Build a route from a lower rack slot to the hub."""
    return track.to_route(rack_node("R", "lower", slot), "hub")


def lower_to_far_bay(slot):
    """Build a route from a lower rack slot to the far right bay."""
    return [("NOW", "ARM_READY", "gripper open, lift down")] + track.to_route(
        rack_node("R", "lower", slot), BAY_NAMES[("R", True)])


def hub_to_upper_entry(slot, holding=True):
    """Build a route from the hub to an upper rack slot."""
    return track.to_route("hub", rack_node("R", "upper", slot), holding=holding)


def upper_exit(slot):
    """Build a route from an upper rack slot back to the hub."""
    return track.to_route(rack_node("R", "upper", slot), "hub")


def slot_visit(deck="lower"):
    """Build the actions for approaching, placing at, and leaving a rack slot."""
    left = C.RACK_TIMEOUT_MS - C.RACK_RAISE_AT_MS[deck]
    approach = "RACK_APPROACH" if deck == "lower" else "RACK_APPROACH_UPPER"
    return [("NOW", approach, "part way up the stub: stop, raise the lift"),
            ("TIMEOUT", "PLACE", "end of the stub: release and turn round", max(500, left))]


def survey_to(start, side, deck, slot):
    """Already-sided route from hub, a checked rack, or an outward-facing bay."""
    destination = rack_node(side, deck, slot)
    if start != "hub" and start not in track.racks and start not in BAY_NAMES.values():
        raise ValueError("Survey must start at hub, a checked rack, or a loading bay")
    route = [("NOW", "ARM_READY", "gripper open, lift down")] if start == "hub" else []
    route += track.to_route(start, destination)
    check = "CHECK_SLOT" if deck == "lower" else "CHECK_SLOT_UPPER"
    return route + [("NOW", check, "check %s and turn round in the stub" % destination)]


def survey_check(deck, slot):
    """Build a survey route from the hub to a rack slot."""
    return survey_to("hub", "R", deck, slot)


def survey_next(deck, slot):
    """Build a survey route from the previous slot to the next one."""
    return survey_to(rack_node("R", deck, slot - 1), "R", deck, slot)


def survey_back(deck, slot):
    """Build a route from a surveyed rack slot back to the hub."""
    return track.to_route(rack_node("R", deck, slot), "hub")
