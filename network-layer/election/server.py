"""gRPC handlers for ElectionService — the thin adapter between the wire and
`election.bully.BullyElection`.

WHERE
    Registered on every bot's server by `bot.Bot._start_grpc_server()`.
    Runs on gRPC worker threads; must not block.

WHY
    Keeps protobuf types out of the election logic so `BullyElection` can be
    unit-tested with plain ints and a fake transport.

HOW
    `Elect` hands the current roster to `handle_elect` so that, if we outrank
    the caller, we can immediately start our own election against the same
    peers. `Coordinator` simply forwards.
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import grpc

from proto import fleet_pb2, fleet_pb2_grpc

if TYPE_CHECKING:
    from election.bully import BullyElection
    from peers.table import PeerTable

log = logging.getLogger(__name__)


class ElectionServicer(fleet_pb2_grpc.ElectionServiceServicer):
    def __init__(self, election: BullyElection, peer_table: PeerTable) -> None:
        self._election = election
        self._peer_table = peer_table

    def Elect(self, request: fleet_pb2.ElectRequest, context: grpc.ServicerContext):
        # The challenger is alive and in our region (the virtual network
        # checked). Register it *before* we take the roster snapshot so that,
        # if we win, our Coordinator reaches it even when it was missing from
        # the last ack we saw. Without this a new leader can win an election
        # and never tell the very bot that triggered it.
        if request.address:
            self._peer_table.ensure(request.bot_id, request.address, request.priority)
        peers = self._peer_table.all_peers()
        ack = self._election.handle_elect(request.bot_id, request.priority, peers)
        return fleet_pb2.ElectResponse(ack=ack)

    def Coordinator(self, request: fleet_pb2.CoordinatorRequest, context: grpc.ServicerContext):
        self._election.handle_coordinator(request.bot_id, request.priority, request.address)
        return fleet_pb2.CoordinatorResponse()
