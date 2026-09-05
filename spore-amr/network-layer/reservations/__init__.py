"""Bot-to-bot reservations — the one channel that does not go through a leader.

See PROTOCOL.md §15. `claims` and `ledger` are the rules, `vicinity` decides who
to tell, `sender` puts announcements on the wire and `server` takes them off it.
"""
from __future__ import annotations

import time


def now_ms() -> int:
    """Local time in milliseconds, for claim windows.

    Monotonic, not wall clock: a claim says "for the next two seconds", and an
    NTP correction mid-window would otherwise stretch or shrink it. Nothing
    compares these numbers across bots — windows travel as offsets precisely so
    that two clocks never have to agree.
    """
    return int(time.monotonic() * 1000)
