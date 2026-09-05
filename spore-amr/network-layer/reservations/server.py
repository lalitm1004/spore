"""gRPC handler for ReservationService — the wire's edge.

WHERE
    Registered on every bot by `bot.Bot._start_grpc_server()`. Runs on gRPC
    worker threads.

WHY
    Keeps protobuf out of `reservations.ledger`, so the rules can be tested with
    plain integers and no server (the same split as `election.server`).

HOW
    Stamps arrival time on the way in. That single line is what lets two bots
    exchange claims without their clocks agreeing: the sender said "+200ms to
    +2400ms", and only the receiver decides what that means (PROTOCOL.md §15).
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import grpc

from proto import fleet_pb2, fleet_pb2_grpc
from reservations import now_ms
from reservations.claims import Announce, Window

if TYPE_CHECKING:
    from reservations.ledger import ReservationLedger

log = logging.getLogger(__name__)


class ReservationServicer(fleet_pb2_grpc.ReservationServiceServicer):
    def __init__(self, ledger: ReservationLedger) -> None:
        self._ledger = ledger

    def Announce(self, request: fleet_pb2.ReservationAnnounce, context: grpc.ServicerContext):
        # The caller is in our region — the virtual network checked (§12).
        self._ledger.receive(
            Announce(
                bot_id=request.bot_id,
                seq=request.seq,
                rank=request.yield_priority,
                ttl_ms=request.ttl_ms,
                windows=tuple(
                    Window(
                        node_id=w.node_id,
                        start_offset_ms=w.start_offset_ms,
                        end_offset_ms=w.end_offset_ms,
                    )
                    for w in request.windows
                ),
            ),
            now_ms(),
        )
        return fleet_pb2.ReservationAck()


def to_proto(announce: Announce) -> fleet_pb2.ReservationAnnounce:
    """Our own announcement, ready to send."""
    return fleet_pb2.ReservationAnnounce(
        bot_id=announce.bot_id,
        seq=announce.seq,
        yield_priority=announce.rank,
        ttl_ms=announce.ttl_ms,
        windows=[
            fleet_pb2.ClaimWindow(
                node_id=w.node_id,
                start_offset_ms=w.start_offset_ms,
                end_offset_ms=w.end_offset_ms,
            )
            for w in announce.windows
        ],
    )
