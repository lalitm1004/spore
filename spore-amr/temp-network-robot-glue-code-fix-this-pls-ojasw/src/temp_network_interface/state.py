"""The network layer's view of the world, and how it stays consistent.

`Fleet` is the authoritative global state: the latest status of every robot and
the commands outstanding against each. It is pure -- the durable copy lives in
the journal (see `store.py`), which replays into a Fleet on startup.

Reconciliation is the rule that keeps the world state honest: a command stays
outstanding until the robot's next status shows it has been carried out, at
which point it is dropped. That is what lets the network layer say, at any
moment, what each robot should be doing and whether it has been done.

Pure: no grpc, no I/O beyond the optional journal handle it logs to.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

from temp_network_interface.messages import NetworkToRobot, RobotToNetwork


@dataclass(frozen=True)
class TargetedCommand:
    """A command addressed to a specific robot.

    `NetworkToRobot` carries no destination -- one stream per robot supplies
    that implicitly on the wire -- so the policy layer names the recipient here,
    and the relay routes by it.
    """

    bot_id: int
    command: NetworkToRobot


def fulfilled(command: NetworkToRobot, status: RobotToNetwork) -> bool:
    """Whether a robot's latest status shows a command has been carried out.

    A mission command is fulfilled once the robot reports it is in that mission
    (and, for cargo, holding that cargo in that state). A bare navigation
    command is fulfilled once the robot reports arrival at the target node.
    """
    if command.set_mission is not None:
        if status.mission.type != command.set_mission.type:
            return False
        cargo = command.set_mission.cargo
        if cargo is not None:
            if status.mission.cargo is None:
                return False
            return (status.mission.cargo.cargo_id == cargo.cargo_id
                    and status.mission.cargo.state == cargo.state)
        return True
    return status.latest_node_id == command.target_node_id


class Fleet:
    """Global world state: latest status and outstanding commands per robot.

    Thread-safe: one connection per robot runs concurrently, and a command for
    robot A may be produced by robot B's message, so mutations arrive from many
    threads. The lock guards the shared maps; each accessor returns a snapshot.
    """

    def __init__(self, journal=None):
        self._journal = journal
        self._lock = threading.RLock()
        self._status: dict[int, RobotToNetwork] = {}
        self._pending: dict[int, list[NetworkToRobot]] = {}

    # ------------------------------------------------------------ mutation --

    def record_status(self, status: RobotToNetwork) -> None:
        """Fold in a status report and journal the event."""
        with self._lock:
            self._apply_status(status)
            self._log({"type": "status", "bot_id": status.bot_id,
                       "status": status.to_dict()})

    def record_command(self, bot_id: int, command: NetworkToRobot) -> None:
        """Queue an outstanding command and journal the event."""
        with self._lock:
            self._apply_command(bot_id, command)
            self._log({"type": "command", "bot_id": bot_id,
                       "command": command.to_dict()})

    # In-memory only; used directly by replay so nothing is journaled twice.
    def update(self, status: RobotToNetwork) -> None:
        with self._lock:
            self._apply_status(status)

    def queue(self, bot_id: int, command: NetworkToRobot) -> None:
        with self._lock:
            self._apply_command(bot_id, command)

    def _apply_status(self, status: RobotToNetwork) -> None:
        self._status[status.bot_id] = status
        remaining = [c for c in self._pending.get(status.bot_id, ())
                     if not fulfilled(c, status)]
        if remaining:
            self._pending[status.bot_id] = remaining
        else:
            self._pending.pop(status.bot_id, None)

    def _apply_command(self, bot_id: int, command: NetworkToRobot) -> None:
        self._pending.setdefault(bot_id, []).append(command)

    def _log(self, record: dict) -> None:
        if self._journal is not None:
            self._journal.append(record)

    # ------------------------------------------------------------- access --

    def robot(self, bot_id: int):
        with self._lock:
            return self._status.get(bot_id)

    def pending(self, bot_id: int) -> list[NetworkToRobot]:
        with self._lock:
            return list(self._pending.get(bot_id, ()))

    def bots(self) -> list[RobotToNetwork]:
        with self._lock:
            return list(self._status.values())

    def __len__(self) -> int:
        with self._lock:
            return len(self._status)

    # ------------------------------------------------------------ restore --

    @classmethod
    def load(cls, journal) -> "Fleet":
        """Rebuild a Fleet by replaying a journal, then keep appending to it.

        The journal must already be open (see `Journal.open`), so the returned
        fleet's future `record_*` calls keep writing to the same file.
        """
        fleet = cls(journal=journal)
        for record in journal.read():
            kind = record["type"]
            if kind == "status":
                fleet.update(RobotToNetwork.from_dict(record["status"]))
            elif kind == "command":
                fleet.queue(record["bot_id"],
                            NetworkToRobot.from_dict(record["command"]))
            else:
                raise ValueError("unknown journal record type {!r}".format(kind))
        return fleet
