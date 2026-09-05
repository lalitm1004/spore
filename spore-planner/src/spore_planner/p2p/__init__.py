"""The peer plane: direct reservation exchange between bots in the vicinity.

Kept separate from `planner` because it is about agreement between robots, not
about routing one of them, and separate from any transport because the messages
here are plain data -- `proto/reservation.proto` shows the gRPC form the network
layer would carry them in.
"""

from spore_planner.p2p.claims import Announcement, Claim, Contest, Window, contests
from spore_planner.p2p.ledger import Ledger
from spore_planner.p2p.vicinity import (
    in_claim_range,
    neighbours,
    radius_for,
    within,
)

__all__ = [
    "Announcement",
    "Claim",
    "Contest",
    "Ledger",
    "Window",
    "contests",
    "in_claim_range",
    "neighbours",
    "radius_for",
    "within",
]
