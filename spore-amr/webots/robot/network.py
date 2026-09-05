"""Stand-in for the network layer, until the real one exists.

The point is not the routing -- it is the interface. Everything above the
`Router` protocol is what the real distributed layer will replace, so the
firmware and the companion get written against their final contract now rather
than being ported later.

`RandomRouter` is honest about being a mock: it picks a legal turn uniformly
and knows nothing about tasks, congestion or other robots. That is enough to
exercise the whole junction path -- arrive, ask, wait, turn, resume -- and a
fleet of them is a real baseline, since random assignment is the floor any
allocation algorithm has to beat.

It picks from the turns the robot says exist, rather than from left/straight/
right unconditionally. A blind choice names a lane that is not there -- at a
corner two of three answers are walls -- and the robot then needs a retry loop
more complicated than the field that avoids it.

Message shapes follow spore-amr/shared/schemas/. Pure: no I/O, no transport.
"""

import json
import random
from dataclasses import dataclass
from typing import Dict, Optional

TURNS = ("left", "straight", "right")


@dataclass(frozen=True)
class Query:
    """What a robot sends on arriving at a node.

    `available` is the turns that lead somewhere, resolved by the robot from
    the shared warehouse map against the heading it arrived on.
    """

    query_id: int
    node_id: int
    node_type: str
    region_id: int
    x_cm: float
    y_cm: float
    heading_rad: float
    available: Dict[str, int]        # turn -> destination node id

    def to_json(self) -> str:
        return json.dumps({
            "schema_version": "v0.1.0",
            "query_id": self.query_id,
            "robot_position": {"x": self.x_cm, "y": self.y_cm},
            "heading_rad": round(self.heading_rad, 5),
            "node": {
                "id": self.node_id,
                "node_type": self.node_type,
                "region_id": self.region_id,
            },
            "available": self.available,
        }, separators=(",", ":"))

    @classmethod
    def from_json(cls, text: str) -> "Query":
        d = json.loads(text)
        node = d["node"]
        return cls(
            query_id=int(d["query_id"]),
            node_id=int(node["id"]),
            node_type=node.get("node_type", "PT"),
            region_id=int(node.get("region_id", 0)),
            x_cm=float(d["robot_position"]["x"]),
            y_cm=float(d["robot_position"]["y"]),
            heading_rad=float(d.get("heading_rad", 0.0)),
            available={k: int(v) for k, v in (d.get("available") or {}).items()},
        )


@dataclass(frozen=True)
class Decision:
    """The answer: which way to turn, and what the robot should arrive at.

    `query_id` comes back so a fresh answer is distinguishable from a late
    answer to the previous question. Two junctions can share a destination -- a
    reroute to the same place -- and that is exactly when confusing them would
    matter.
    """

    query_id: int
    turn: str
    target_node_id: int

    def to_json(self) -> str:
        return json.dumps({
            "schema_version": "v0.1.0",
            "query_id": self.query_id,
            "turn": self.turn,
            "target_node_id": self.target_node_id,
        }, separators=(",", ":"))

    @classmethod
    def from_json(cls, text: str) -> "Decision":
        d = json.loads(text)
        turn = d["turn"]
        if turn not in TURNS:
            raise ValueError("unknown turn {!r}".format(turn))
        return cls(query_id=int(d["query_id"]), turn=turn,
                   target_node_id=int(d["target_node_id"]))


class RandomRouter:
    """Picks a legal turn at random.

    Seeded, because a run that cannot be reproduced cannot be debugged at 3am
    and cannot honestly be averaged over trials.
    """

    name = "random"

    def __init__(self, seed: int = 0):
        self.random = random.Random(seed)
        self.decisions = 0

    def route(self, query: Query) -> Optional[Decision]:
        if not query.available:
            # A dead end. The caller decides what to do about it; the router
            # has nothing legal to offer and will not invent something.
            return None

        turn = self.random.choice(sorted(query.available))
        self.decisions += 1
        return Decision(query_id=query.query_id, turn=turn,
                        target_node_id=query.available[turn])
