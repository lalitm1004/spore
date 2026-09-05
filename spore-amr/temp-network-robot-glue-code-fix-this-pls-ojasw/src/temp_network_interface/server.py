"""The network layer's gRPC server.

Accepts one bidirectional stream per robot. Incoming `RobotToNetwork` messages
are persisted into the durable fleet and handed to a policy; the policy returns
`TargetedCommand`s, which are persisted and relayed to the destination robot's
stream -- including streams other than the one whose message triggered the
decision.

Each connection runs an event loop on the handler's generator: a consumer
thread drains the request stream into an inbox, and the loop processes the inbox
and yields any commands queued for this robot. That is what lets a command reach
a robot that is idle rather than reporting, and it is why the bidi stream is the
right shape: delivery is push, not request/response.

Command delivery is at-least-once. A command is persisted, delivered if the
robot is connected, and left outstanding in the fleet until that robot's next
status reconciles it; if the robot reconnects before then, the command is
delivered again. The stubs' commands are idempotent, so a duplicate is harmless;
exactly-once needs correlation ids, which the schema deliberately does not carry.
"""

from __future__ import annotations

import queue
import threading
from concurrent import futures

import grpc

from temp_network_interface import network_pb2_grpc
from temp_network_interface.messages import RobotToNetwork
from temp_network_interface.policy import HoldPolicy
from temp_network_interface.relay import Relay
from temp_network_interface.state import Fleet
from temp_network_interface.store import Journal
from temp_network_interface.transport import decode, encode_network_to_robot

# How long a connection loop blocks waiting for an outbound command before it
# re-checks the inbox and the stream's liveness. Commands arrive on the order of
# seconds in a warehouse, so this is far below what matters.
_POLL_INTERVAL_S = 0.05


class NetworkService(network_pb2_grpc.RobotNetworkServicer):
    def __init__(self, *, fleet=None, journal=None, policy=None, relay=None):
        if fleet is not None:
            self.fleet = fleet
        elif journal is not None:
            self.fleet = Fleet.load(journal)
        else:
            self.fleet = Fleet()
        self.policy = policy if policy is not None else HoldPolicy()
        self.relay = relay if relay is not None else Relay()

    def Session(self, request_iterator, context):
        inbox: "queue.Queue" = queue.Queue()
        outbox: "queue.Queue" = queue.Queue()
        done = threading.Event()
        bot_id = None

        def consume():
            try:
                for envelope in request_iterator:
                    inbox.put(envelope)
            except Exception:
                pass  # the stream ended or was cancelled; that is not an error
            finally:
                done.set()

        threading.Thread(target=consume, daemon=True).start()

        try:
            while True:
                # 1. Fold in everything the robot has reported so far.
                while True:
                    try:
                        envelope = inbox.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        message = decode(envelope)
                    except Exception:
                        continue
                    if not isinstance(message, RobotToNetwork):
                        continue

                    if bot_id is None:
                        bot_id = message.bot_id
                        self.relay.attach(bot_id, outbox)
                        for command in self.fleet.pending(bot_id):
                            self.relay.deliver(bot_id, command)

                    self.fleet.record_status(message)
                    for targeted in self.policy.on_status(self.fleet, message):
                        self.fleet.record_command(targeted.bot_id, targeted.command)
                        self.relay.deliver(targeted.bot_id, targeted.command)

                # 2. Emit any command addressed to this robot, blocking briefly
                # so a command from another robot can arrive while this one is
                # not reporting.
                try:
                    command = outbox.get(timeout=_POLL_INTERVAL_S)
                except queue.Empty:
                    if done.is_set() and inbox.empty():
                        break
                    continue
                yield encode_network_to_robot(command)
        finally:
            if bot_id is not None:
                self.relay.detach(bot_id)


def serve(address: str = "[::]:50051", journal=None, policy=None) -> None:
    """Run the network service until interrupted.

    `journal` is an optional path to the durable state file; when given, the
    fleet is rebuilt from it on startup and kept appending to it.
    """
    store = Journal(journal).open() if journal else None
    service = NetworkService(journal=store, policy=policy)
    server = grpc.server(futures.ThreadPoolExecutor())
    network_pb2_grpc.add_RobotNetworkServicer_to_server(service, server)
    server.add_insecure_port(address)
    server.start()
    print("temp-network-interface listening on {}".format(address), flush=True)
    server.wait_for_termination()
