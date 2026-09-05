"""Delivering commands to the robots that should receive them.

The wire contract has one stream per robot, so `NetworkToRobot` carries no
destination -- the stream is the destination. `Relay` is the routing table that
turns a `TargetedCommand`'s `bot_id` into that stream's outbound queue, so a
policy can command any robot, not just the one whose message triggered it.

A robot with no live connection has its command left outstanding in the `Fleet`
and delivered when it next connects.

Pure: no grpc, just queues.
"""

from __future__ import annotations

import queue
import threading

from temp_network_interface.messages import NetworkToRobot


class Relay:
    def __init__(self):
        self._lock = threading.Lock()
        self._connections: dict[int, "queue.Queue"] = {}

    def attach(self, bot_id: int, outbox: "queue.Queue") -> None:
        with self._lock:
            self._connections[bot_id] = outbox

    def detach(self, bot_id: int) -> None:
        with self._lock:
            self._connections.pop(bot_id, None)

    def deliver(self, bot_id: int, command: NetworkToRobot) -> bool:
        """Queue a command to a connected robot.

        Returns False if the robot has no live connection, meaning the command
        stays pending in the fleet until it next connects.
        """
        with self._lock:
            outbox = self._connections.get(bot_id)
        if outbox is None:
            return False
        outbox.put(command)
        return True
