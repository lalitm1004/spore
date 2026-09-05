"""Stand-in for the network layer, until the real one exists.

The point is not the routing -- it is the interface. Everything above the
`Router` protocol is what the real distributed layer will replace, so the
firmware and the companion get written against their final contract now rather
than being ported later.

That contract is `spore-amr/shared/schemas/`, and it is narrower than it looks.
A robot sends `latest_node_id`: where it is. It receives `target_node_id`:
where to go next. Both schemas set `additionalProperties: false`, so there is
no room for the robot to offer a menu of turns, and none for the answer to name
a direction. Left and right never cross the wire.

That division is the right one anyway. **The network layer holds the map and
chooses the route; the robot holds the map and drives it.** A robot told "go to
node 70" works out the bearing itself, which it can do exactly, because lanes
are straight and it knows where both nodes are. Being told "turn left" would
add nothing it could not derive and would put a second, weaker description of
the geometry on the wire.

`RandomRouter` is honest about being a mock: it picks a neighbour uniformly and
knows nothing about tasks, congestion or other robots. That is enough to
exercise the whole junction path -- arrive, ask, wait, turn, resume -- and a
fleet of them is a real baseline, since random assignment is the floor any
allocation algorithm has to beat.

Pure: no I/O, no transport.
"""

import json
import random
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class Query:
    """What a robot sends on arriving at a node: `RobotToNetwork`.

    Position is not in it, and neither are the turns that exist. The network
    layer knows the map; `latest_node_id` is enough to place the robot on it,
    and it is the only localisation claim a marker read actually supports.
    """

    bot_id: int
    region_id: int
    latest_node_id: int
    battery_percent: float = 100.0
    mission: str = "IDLE"
    timestamp: int = 0

    def to_json(self) -> str:
        return json.dumps({
            "bot_id": self.bot_id,
            "region_id": self.region_id,
            "latest_node_id": self.latest_node_id,
            "mission": {"type": self.mission},
            "telemetry": {"battery": {"percentage": self.battery_percent}},
            "timestamp": self.timestamp,
        }, separators=(",", ":"))

    @classmethod
    def from_json(cls, text: str) -> "Query":
        d = json.loads(text)
        telemetry = d.get("telemetry") or {}
        battery = telemetry.get("battery") or {}
        mission = d.get("mission") or {}
        return cls(
            bot_id=int(d["bot_id"]),
            region_id=int(d.get("region_id", 0)),
            latest_node_id=int(d["latest_node_id"]),
            battery_percent=float(battery.get("percentage", 100.0)),
            mission=str(mission.get("type", "IDLE")),
            timestamp=int(d.get("timestamp", 0)),
        )


@dataclass(frozen=True)
class Decision:
    """The answer: `NetworkToRobot`. Which node to head for, and nothing else.

    `timestamp` is echoed back. It is the only field in the schema that can
    distinguish a fresh answer from a late answer to the previous junction,
    and two junctions can share a destination -- a reroute to the same place --
    which is exactly when confusing them would matter.
    """

    target_node_id: int
    timestamp: int = 0
    set_mission: Optional[str] = None

    def to_json(self) -> str:
        payload = {
            "target_node_id": self.target_node_id,
            "timestamp": self.timestamp,
        }
        if self.set_mission is not None:
            payload["set_mission"] = {"type": self.set_mission}
        return json.dumps(payload, separators=(",", ":"))

    @classmethod
    def from_json(cls, text: str) -> "Decision":
        d = json.loads(text)
        if "target_node_id" not in d:
            raise ValueError("a decision must name a target node")
        mission = d.get("set_mission") or None
        return cls(
            target_node_id=int(d["target_node_id"]),
            timestamp=int(d.get("timestamp", 0)),
            set_mission=str(mission["type"]) if mission else None,
        )


class RandomRouter:
    """Picks a neighbouring node at random.

    Holds the map, because choosing a route is what it is for and the schema
    gives it nothing else to choose from. That also makes a dead end ordinary
    rather than special: a charging bay has exactly one neighbour, so the only
    thing to pick is the way back to the corridor. While the choice was made
    from a menu of left/straight/right, the way back matched none of them and
    robots routed into a bay stayed there.

    Seeded, because a run that cannot be reproduced cannot be debugged at 3am
    and cannot honestly be averaged over trials.
    """

    name = "random"

    def __init__(self, graph, seed: int = 0):
        self.graph = graph
        self.random = random.Random(seed)
        self.decisions = 0
        self._at: Dict[int, int] = {}
        self._came_from: Dict[int, int] = {}

    def route(self, query: Query) -> Optional[Decision]:
        if query.latest_node_id not in self.graph.nodes:
            # Not somewhere this router knows. It will not invent a target.
            return None

        neighbours = sorted(self.graph.neighbours(query.latest_node_id))
        if not neighbours:
            return None

        # Don't send it straight back where it came from. A walk that may undo
        # its last move covers ground far more slowly and turns every corridor
        # into a head-on meeting, and the robot used to be the one enforcing
        # this -- it filtered the arrival lane out of the menu it offered.
        # Routing is this side's job now, and no extra field is needed to do
        # it: whoever answered the last question knows where the robot was.
        #
        # Avoided, not forbidden. A degree-1 charging bay has one neighbour and
        # it is the way in; refusing it would strand the robot there.
        # History only advances when the robot has actually moved. It can be
        # asked about the same node twice -- the obstacle reflex retreats it to
        # the marker it came from, and it reads that one again -- and treating
        # that as a move would forget where it came from and offer the way back.
        if self._at.get(query.bot_id) != query.latest_node_id:
            self._came_from[query.bot_id] = self._at.get(query.bot_id)
            self._at[query.bot_id] = query.latest_node_id

        previous = self._came_from.get(query.bot_id)
        forward = [n for n in neighbours if n != previous] or neighbours

        target = self.random.choice(forward)
        self.decisions += 1
        return Decision(target_node_id=target, timestamp=query.timestamp)
