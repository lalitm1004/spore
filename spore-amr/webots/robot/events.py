"""What the firmware reports upward, and when.

The companion never polls the sensors; it learns about the world from these
messages. Edge-triggered for conditions, periodic for status. Pure: no I/O.
"""

from typing import List, Optional

from robot.protocol import Message


class EventDetector:
    """Edge-triggered reports, with a debounce under the lost-line edge.

    `lost_debounce_s` is how long the line must stay absent before it counts as
    lost. Without it a single 16 ms dropout was a LINE_LOST, and since the
    companion answers every LINE_LOST by cutting speed, a handful of blips left
    a robot crawling at the floor for the rest of the run. The genuine
    lost-line case is unaffected: it is seconds long, and the firmware's own
    halt timeout is independent of this event.
    """

    def __init__(self, status_period_s: float = 1.0, lost_debounce_s: float = 0.0):
        self.status_period_s = status_period_s
        self.lost_debounce_s = lost_debounce_s
        self._reported_lost = False
        self._lost_since: Optional[float] = None
        self._next_status_at: Optional[float] = None

    def update(self, t: float, lost: bool, error: float, u: float) -> List[Message]:
        messages: List[Message] = []

        if lost:
            if self._lost_since is None:
                self._lost_since = t
            if (not self._reported_lost
                    and t - self._lost_since >= self.lost_debounce_s):
                self._reported_lost = True
                messages.append(Message(kind="EVT", name="LINE_LOST",
                                        fields={"t": round(t, 4)}))
        else:
            self._lost_since = None
            # Only report a reacquisition of a loss the companion heard about.
            # A debounced blip otherwise produces a LINE_FOUND for a LINE_LOST
            # that was never sent.
            if self._reported_lost:
                self._reported_lost = False
                messages.append(Message(kind="EVT", name="LINE_FOUND",
                                        fields={"t": round(t, 4)}))

        if self._next_status_at is None or t >= self._next_status_at:
            messages.append(
                Message(
                    kind="EVT",
                    name="STATUS",
                    fields={
                        "t": round(t, 4),
                        "err": round(error, 6),
                        "u": round(u, 4),
                        "lost": int(lost),
                    },
                )
            )
            self._next_status_at = t + self.status_period_s

        return messages
