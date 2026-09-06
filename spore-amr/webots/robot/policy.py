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
        laden_speed: float = None,
    ):
        # Empty robots run fast; loaded ones run at the speed the line follower
        # was tuned for. A real AMR does the same thing, and here it is close to
        # free: a job that takes fewer simulated seconds takes fewer ticks, and
        # ticks are the only thing the simulator actually spends.
        #
        # `laden_speed` defaults to the unladen speed, which keeps the
        # behaviour of every caller that does not set it. Nothing sets the
        # mission today -- the network layer never sends `set_mission`, so
        # `laden` below is never true and this is the fast path throughout.
        # The mechanism is here so that the day cargo state does cross the
        # wire, the robot already slows for it.
        self.cruise_speed = cruise_speed
        self.laden_speed = cruise_speed if laden_speed is None else laden_speed
        self._laden = False
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

    @property
    def target_cruise(self) -> float:
        """The speed this robot should be doing when the line is clean."""
        return self.laden_speed if self._laden else self.cruise_speed

    def set_laden(self, laden: bool) -> List[Message]:
        """Tell the policy whether the robot is carrying cargo.

        Changing the ceiling does not reach past a throttle that is already
        holding the robot below it: a robot slowed for a lost line stays slowed,
        and recovery climbs to whichever ceiling now applies.
        """
        if laden == self._laden:
            return []
        self._laden = laden
        self._target_speed = min(self._target_speed, self.target_cruise) \
            if laden else min(max(self._target_speed, self.min_speed),
                              self.target_cruise)
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
                and self._target_speed < self.target_cruise):
            if self._clean_since is None:
                self._clean_since = now
            elif now - self._clean_since >= self.recover_after_s:
                self._clean_since = now
                self._target_speed = min(self.target_cruise,
                                         self._target_speed * self.speedup)
                return [self._set_speed()]

        return []

    def _set_speed(self) -> Message:
        return Message(kind="CMD", name="SET_SPEED", fields={"value": self._target_speed})
