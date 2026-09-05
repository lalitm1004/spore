"""The companion's decision making -- the Pi Zero's job.

It never touches a sensor or a motor. It watches events and issues commands,
which is the division that has to be right before any of this meets hardware:
if the link stalls, the firmware keeps following the line on its own.
Pure: no I/O.
"""

from typing import List

from robot.protocol import Message


class CompanionPolicy:
    def __init__(
        self,
        cruise_speed: float,
        min_speed: float,
        slowdown: float,
        mission_duration_s: float,
    ):
        self.cruise_speed = cruise_speed
        self.min_speed = min_speed
        self.slowdown = slowdown
        self.mission_duration_s = mission_duration_s
        self._target_speed = cruise_speed
        self._stopped = False

    def start(self) -> List[Message]:
        return [self._set_speed()]

    def on_event(self, message: Message) -> List[Message]:
        if self._stopped:
            return []

        if message.name == "LINE_LOST":
            # Losing the line means we were asking too much of the tight loop.
            self._target_speed = max(self.min_speed, self._target_speed * self.slowdown)
            return [self._set_speed()]

        if message.fields.get("t", 0.0) >= self.mission_duration_s:
            self._stopped = True
            return [Message(kind="CMD", name="STOP", fields={})]

        return []

    def _set_speed(self) -> Message:
        return Message(kind="CMD", name="SET_SPEED", fields={"value": self._target_speed})
