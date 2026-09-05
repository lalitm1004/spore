"""The robot <-> network wire: what a robot asks, and what it is told.

At every QR node the companion resolves which turns physically exist and asks
this question; the network layer answers with one of them. The robot never
picks a direction for itself -- that is the local autonomy the architecture
says the network layer owns.

The network layer is `spore-amr/network-layer`, one bot process per robot,
serving these messages on a unix socket (its PROTOCOL.md §16). This module is
only the message shapes, shared by both sides so neither can drift: pure, no
I/O, no transport.

Message shapes follow spore-amr/shared/schemas/.
"""

import json
from dataclasses import dataclass
from typing import Dict, Optional

TURNS = ("left", "straight", "right")

# What an answer can be. The original protocol had only "take this lane", which
# left no way to say "stay where you are" -- and a robot told nothing is
# indistinguishable from a robot whose network layer has died, because it only
# asks again on reaching the next node. WAIT closes that hole: hold for
# `hold_ms`, then ask the same question again.
PROCEED, REROUTE, WAIT, YIELD = "PROCEED", "REROUTE", "WAIT", "YIELD"
KINDS = (PROCEED, REROUTE, WAIT, YIELD)


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
    kind: str = PROCEED
    hold_ms: int = 0
    because: str = ""

    @property
    def is_wait(self) -> bool:
        return self.kind == WAIT

    def to_json(self) -> str:
        return json.dumps({
            "schema_version": "v0.1.0",
            "query_id": self.query_id,
            "kind": self.kind,
            "turn": self.turn,
            "target_node_id": self.target_node_id,
            "hold_ms": self.hold_ms,
            "because": self.because,
        }, separators=(",", ":"))

    @classmethod
    def from_json(cls, text: str) -> "Decision":
        d = json.loads(text)
        # `kind` is optional so an older network layer, which sent only a turn,
        # still parses: no kind means it is telling us to move.
        kind = str(d.get("kind", PROCEED))
        if kind not in KINDS:
            raise ValueError("unknown kind {!r}".format(kind))
        turn = d.get("turn", "")
        # WAIT names no lane. Every other kind must name one that exists, or the
        # robot would turn to a bearing nobody computed.
        if kind != WAIT and turn not in TURNS:
            raise ValueError("unknown turn {!r}".format(turn))
        return cls(query_id=int(d["query_id"]), turn=turn,
                   target_node_id=int(d.get("target_node_id", 0)),
                   kind=kind, hold_ms=int(d.get("hold_ms", 0)),
                   because=str(d.get("because", "")))
