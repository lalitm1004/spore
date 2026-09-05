"""The network layer's decision logic, kept separate from the transport.

A policy maps the fleet's state onto commands. It is a plain callable on
`(fleet, status) -> list[TargetedCommand]`, so the real task-allocation layer can
slot in later without touching the gRPC server. The two policies here are stubs:
they exist to exercise the interface and to give the fleet a reproducible
baseline, in the same spirit as `robot/network.py`'s `RandomRouter`.

Pure: no grpc, no I/O.
"""

from __future__ import annotations

from typing import Protocol

from temp_network_interface.messages import Mission, NetworkToRobot, RobotToNetwork
from temp_network_interface.state import Fleet, TargetedCommand


class Policy(Protocol):
    def on_status(
        self, fleet: Fleet, status: RobotToNetwork
    ) -> list[TargetedCommand]: ...


class HoldPolicy:
    """Deterministic stub: tell every robot to hold on the node it last saw.

    Enough to exercise the full round trip (status up, command back) and to make
    the interface observable. Replaced by real task allocation later.
    """

    name = "hold"

    def on_status(self, fleet: Fleet, status: RobotToNetwork) -> list[TargetedCommand]:
        return [
            TargetedCommand(
                bot_id=status.bot_id,
                command=NetworkToRobot(
                    target_node_id=status.latest_node_id,
                    set_mission=Mission(type="HOLD"),
                    timestamp=status.timestamp,
                ),
            )
        ]


class NoopPolicy:
    """A policy that issues no commands; robots simply report and keep going."""

    name = "noop"

    def on_status(self, fleet: Fleet, status: RobotToNetwork) -> list[TargetedCommand]:
        return []
