"""The companion's link to the network layer.

A thin adapter over `temp_network_interface.NetworkClient`: it turns "I have
arrived at node 7" into a status report, and reports back the destination the
network layer currently wants this robot at.

The shape is theirs, not ours. Their client is a long-lived bidirectional gRPC
stream with the two directions decoupled by queues, so a robot's control loop
never blocks on the network and an absent network degrades to no-ops. What a
junction needs, though, is an answer *now* -- so this is where the asynchronous
stream is turned back into a bounded wait, and only here.

The wait is bounded twice over: once by `wait_s`, and again by the firmware's
own `junction_timeout_s`, which drives on regardless if nobody answers. A robot
must never hold a lane because a service is slow.

Pure apart from the client it is handed: `grpc` lives behind `NetworkClient`,
which is why this module is testable with a fake.
"""

import time
from typing import Optional

from temp_network_interface.messages import (
    Battery,
    Mission,
    RobotToNetwork,
    Telemetry,
)


class Uplink:
    """Reports position upward; returns the standing destination.

    "Standing" is the important part. `NetworkToRobot` names a node, not a
    turn, and the client preserves the latest one in `state` -- so between
    commands the robot still knows where it is going. A robot is sent across
    the warehouse once rather than steered corner by corner.
    """

    def __init__(self, client, wait_s: float = 5.0, battery_percent: float = 100.0):
        self.client = client
        self.wait_s = wait_s
        self.battery_percent = battery_percent

    def report(self, bot_id: int, region_id: int, node_id: int,
               timestamp: int) -> Optional[int]:
        """Send a status and return where to head for, or None.

        None means nobody has given this robot a destination yet, or the one it
        has is where it already stands and no replacement arrived in time. The
        caller sits still rather than inventing somewhere to go.
        """
        self.client.send(RobotToNetwork(
            bot_id=bot_id,
            region_id=region_id,
            latest_node_id=node_id,
            mission=Mission(type="IDLE"),
            telemetry=Telemetry(battery=Battery(percentage=self.battery_percent)),
            timestamp=timestamp,
        ))

        # A goal equal to where we stand is an arrival, not a destination. The
        # network layer reconciles that from the status just sent and issues a
        # new one, so wait for it rather than reporting "nowhere to go" --
        # otherwise a robot stops for good the moment it first arrives.
        goal = self._usable_goal(node_id)
        if goal is not None:
            return goal

        # Drain, don't sleep: each command the reader thread delivers updates
        # `state`, so one recv is one chance at a fresh destination. Commands
        # that are not one -- a mission change, a repeat of where we stand --
        # cost a turn of the loop rather than the whole wait, but the deadline
        # is absolute either way. A robot must never hold a lane because a
        # service is chatty any more than because it is slow.
        deadline = time.monotonic() + self.wait_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            if self.client.recv(timeout=remaining) is None:
                return None          # timed out, or the stream ended
            goal = self._usable_goal(node_id)
            if goal is not None:
                return goal

    def _usable_goal(self, node_id: int) -> Optional[int]:
        goal = getattr(self.client.state, "target_node_id", None)
        return None if goal is None or goal == node_id else goal
