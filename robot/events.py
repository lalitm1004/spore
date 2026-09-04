"""What the firmware reports upward, and when.

The companion never polls the sensors; it learns about the world from these
messages. Edge-triggered for conditions, periodic for status. Pure: no I/O.
"""

from typing import List, Optional

from robot.protocol import Message


class EventDetector:
    def __init__(self, status_period_s: float = 1.0):
        self.status_period_s = status_period_s
        self._was_lost: Optional[bool] = None
        self._next_status_at: Optional[float] = None

    def update(self, t: float, lost: bool, error: float, u: float) -> List[Message]:
        messages: List[Message] = []

        if self._was_lost is not None and lost != self._was_lost:
            messages.append(
                Message(
                    kind="EVT",
                    name="LINE_LOST" if lost else "LINE_FOUND",
                    fields={"t": round(t, 4)},
                )
            )
        elif self._was_lost is None and lost:
            messages.append(Message(kind="EVT", name="LINE_LOST", fields={"t": round(t, 4)}))
        self._was_lost = lost

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
