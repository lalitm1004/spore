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
        recover_after_s: float = 0.0,
        speedup: float = 1.0,
    ):
        self.cruise_speed = cruise_speed
        self.min_speed = min_speed
        self.slowdown = slowdown
        self.mission_duration_s = mission_duration_s
        # Slowing down was a one-way ratchet: nothing ever raised the speed
        # again, so one transient early in a run left the robot at the floor
        # for all of it -- measured at 1.5 rad/s, a quarter of cruise, for the
        # remaining four minutes. Slowing is a response to conditions, and
        # conditions pass. `recover_after_s` of clean line wins a step back.
        self.recover_after_s = recover_after_s
        self.speedup = speedup
        self._target_speed = cruise_speed
        self._clean_since = None
        self._stopped = False

    def start(self) -> List[Message]:
        return [self._set_speed()]

    def on_event(self, message: Message) -> List[Message]:
        if self._stopped:
            return []

        now = message.fields.get("t", 0.0)

        if message.name == "LINE_LOST":
            # Losing the line means we were asking too much of the tight loop.
            self._target_speed = max(self.min_speed, self._target_speed * self.slowdown)
            # The clean run is measured from the last trouble, not from the
            # next status message -- otherwise the clock starts up to a whole
            # status period late and recovery lags the conditions it tracks.
            self._clean_since = now
            return [self._set_speed()]

        if now >= self.mission_duration_s:
            self._stopped = True
            return [Message(kind="CMD", name="STOP", fields={})]

        if (self.recover_after_s > 0.0
                and self._target_speed < self.cruise_speed):
            if self._clean_since is None:
                self._clean_since = now
            elif now - self._clean_since >= self.recover_after_s:
                self._clean_since = now
                self._target_speed = min(self.cruise_speed,
                                         self._target_speed * self.speedup)
                return [self._set_speed()]

        return []

    def _set_speed(self) -> Message:
        return Message(kind="CMD", name="SET_SPEED", fields={"value": self._target_speed})
