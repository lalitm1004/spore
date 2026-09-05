"""Giving each robot somewhere to be.

`HoldPolicy` tells a robot to stay where it is: enough to prove the round trip,
not enough to move a fleet. This is the first policy that does real work. Each
robot is handed a destination far away and keeps it until it gets there;
reconciliation drops the command on arrival, and the next status earns a new
one. A robot therefore always has exactly one standing goal.

**Far is measured in hops along the lanes, never in metres.** Two nodes either
side of a rack are close in a straight line and a long drive apart, so a
destination chosen on straight-line distance is sometimes a trip round a
corner. On the real warehouse the furthest node from a charging bay is 78 hops
-- about 156 m of driving -- and 427 nodes sit at least 40 hops out.

The map is here because routing is what a network layer is for. The robot is
told a node, not a direction; it holds the map too and walks there itself, one
lane at a time. That division is the schemas' own: `NetworkToRobot` carries
`target_node_id` and has no field for a turn.

Still a stub in one respect, and honestly so: destinations are uniform random
among the far ones. It knows nothing about tasks, congestion, or what any other
robot is doing, which is the seam a real allocator replaces.

Pure: no grpc, no I/O.
"""

from __future__ import annotations

import random as _random
from typing import Optional

from temp_network_interface.messages import NetworkToRobot, RobotToNetwork
from temp_network_interface.state import Fleet, TargetedCommand

# Far enough that getting there is a journey across the warehouse rather than a
# nudge down the aisle. At 2 m node spacing this is 80 m of driving.
DEFAULT_MINIMUM_HOPS = 40


class GoalPolicy:
    """Sends every robot to a random node far from wherever it reports in.

    Seeded, because a run that cannot be reproduced cannot be debugged and
    cannot honestly be averaged over trials -- the same reason the Webots
    stand-in router is seeded.
    """

    name = "goal"

    def __init__(self, graph, minimum_hops: int = DEFAULT_MINIMUM_HOPS,
                 random: Optional[_random.Random] = None):
        self.graph = graph
        self.minimum_hops = minimum_hops
        self.random = random if random is not None else _random.Random(0)

    def on_status(self, fleet: Fleet, status: RobotToNetwork) -> list[TargetedCommand]:
        # A robot already working towards somewhere is left alone. Re-issuing
        # on every status would move the destination at every marker, and the
        # robot would set off afresh for ever without arriving anywhere.
        if fleet.pending(status.bot_id):
            return []

        candidates = self.graph.far_nodes(status.latest_node_id,
                                          minimum_hops=self.minimum_hops)
        if not candidates:
            # Nowhere is *that* far. `minimum_hops` is a preference, not a
            # promise: on the 83-node window nothing is more than 20 hops from
            # a charging bay, and a fleet that sits still because the map is
            # small is worse than one sent as far as the map allows. Fall back
            # to whatever is reachable and let the choice below pick from it.
            candidates = self.graph.far_nodes(status.latest_node_id,
                                              minimum_hops=1)
        if not candidates:
            # Genuinely nowhere to go: an island, or a one-node map. Saying
            # nothing beats naming somewhere unreachable.
            return []

        return [
            TargetedCommand(
                bot_id=status.bot_id,
                command=NetworkToRobot(
                    target_node_id=self.random.choice(candidates),
                    timestamp=status.timestamp,
                ),
            )
        ]
