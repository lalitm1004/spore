"""Stand-in for the network layer, until the real one exists.

The point is not the routing -- it is the interface. Everything above the
`Router` protocol is what the real distributed layer will replace, so the
firmware and the companion get written against their final contract now
instead of being ported later.

`RandomRouter` is honest about being a mock: it picks a legal out-edge
uniformly and knows nothing about tasks, congestion or other robots. That is
enough to exercise the whole junction path -- query, wait, turn, resume -- and
a fleet of them is a real baseline, since "random assignment" is the floor any
allocation algorithm has to beat.

Pure: no I/O, no Webots, no transport.
"""

import random
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class Junction:
    """What the firmware knows when it stops on a node and asks."""

    query_id: int
    node: int
    kind: str
    x_mm: int
    y_mm: int
    out_edges: Tuple[Tuple[int, int], ...]   # (bearing_deg, to_node)
    heading_rad: float = 0.0


@dataclass(frozen=True)
class Route:
    """The answer: turn to this bearing, expect to arrive at this node."""

    query_id: int
    to_node: int
    bearing_deg: int


class RandomRouter:
    """Picks a legal out-edge at random.

    Seeded, because a run that cannot be reproduced cannot be debugged at 3am
    and cannot be averaged over trials honestly.
    """

    name = "random"

    def __init__(self, seed: int = 0, avoid_reversing: bool = True):
        self.random = random.Random(seed)
        self.avoid_reversing = avoid_reversing
        self.decisions = 0

    def route(self, junction: Junction) -> Optional[Route]:
        if not junction.out_edges:
            return None

        choices: Sequence[Tuple[int, int]] = junction.out_edges
        if self.avoid_reversing and len(choices) > 1:
            # A 180 degree turn is legal but reads as a robot changing its mind
            # on the spot, and on a one-way lane it would be a wrong-way entry.
            # Only drop it if something else remains.
            arrived_on = int(round(_degrees(junction.heading_rad))) % 360
            back = (arrived_on + 180) % 360
            forward = [e for e in choices if _bearing_gap(e[0], back) > 45]
            if forward:
                choices = forward

        bearing, to_node = self.random.choice(list(choices))
        self.decisions += 1
        return Route(query_id=junction.query_id, to_node=to_node, bearing_deg=bearing)


def _degrees(radians: float) -> float:
    import math

    return math.degrees(radians)


def _bearing_gap(a: int, b: int) -> int:
    """Smallest angle between two bearings, in degrees."""
    gap = abs(int(a) - int(b)) % 360
    return min(gap, 360 - gap)
