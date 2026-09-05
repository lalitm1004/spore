"""Orders entering the fleet, and the copy of the contract that lets them.

`spore-control-plane` owns `controlplane.proto`; we implement it. The
dependency points one way on purpose — an order system has no business knowing
about `owner_region`, `assignee` or `status`, which is what it would have to
learn if it spoke `fleet.Job`.

The cost of that is a second copy of the file, in `proto/`, because the Docker
build context is this project alone and the sibling is not in it. A copy that
nobody checks is a copy that drifts, so the first test here checks it.
"""
from __future__ import annotations

import pathlib

import grpc
import pytest

from proto import controlplane_pb2, controlplane_pb2_grpc, fleet_pb2
from tests.conftest import md, wait_until

CONTROL_PLANE = (pathlib.Path(__file__).resolve().parents[3]
                 / "spore-control-plane" / "proto" / "controlplane.proto")
VENDORED = pathlib.Path(__file__).resolve().parents[1] / "proto" / "controlplane.proto"


def _stub(bot):
    return controlplane_pb2_grpc.ControlPlaneServiceStub(
        grpc.insecure_channel(bot.address))


def _order(order_id: str, pickup: int, dropoff: int) -> controlplane_pb2.Order:
    return controlplane_pb2.Order(
        order_id=order_id, pickup_node=pickup, dropoff_node=dropoff)


# ---- the contract ------------------------------------------------------------

def test_the_vendored_proto_matches_the_control_planes():
    """Their file is the original. Ours is a copy because the build context is
    this directory, and a copy nobody compares is a copy that rots."""
    if not CONTROL_PLANE.exists():
        pytest.skip("the control plane is not checked out beside us")
    assert VENDORED.read_text() == CONTROL_PLANE.read_text(), (
        f"{VENDORED} has drifted from {CONTROL_PLANE}; copy it across and "
        "regenerate the stubs (see README.md)")


def test_dispatch_ack_says_the_same_things_a_job_ack_does():
    """The adapter is a translation and nothing else, which only holds while the
    two acks carry the same four answers. A field added to one and not the other
    is a fact the control plane would silently stop being told."""
    ours = {f.name for f in fleet_pb2.JobAck.DESCRIPTOR.fields}
    theirs = {f.name for f in controlplane_pb2.DispatchAck.DESCRIPTOR.fields}
    assert theirs <= ours, f"DispatchAck asks for {sorted(theirs - ours)}"


# ---- placing an order --------------------------------------------------------

def test_an_order_reaches_a_free_bot(fleet):
    """The whole point: an order goes to *any* bot and the fleet works out
    whose it is. The control plane names no leader and no region."""
    leader, follower = fleet.bot(2), fleet.bot(1)
    fleet.link_all()
    leader.become_leader()
    follower.become_follower(leader.bot_id, leader.address)
    assert wait_until(lambda: leader.peer_table.get(1) is not None, 5)

    ack = _stub(leader).DispatchOrder(
        _order("order-1", 3, 9), timeout=5, metadata=md(999, 0, "orders"))

    assert ack.accepted, ack.note


def test_an_order_submitted_to_a_follower_is_still_placed(fleet):
    """Every bot is a door in, because the control plane cannot know which one
    leads — leaders rotate, and anything it cached would be wrong by the time it
    was used. A non-leader forwards."""
    leader, follower = fleet.bot(2), fleet.bot(1)
    fleet.link_all()
    leader.become_leader()
    follower.become_follower(leader.bot_id, leader.address)
    assert wait_until(lambda: leader.peer_table.get(1) is not None, 5)

    ack = _stub(follower).DispatchOrder(
        _order("order-2", 3, 9), timeout=5, metadata=md(999, 0, "orders"))

    assert ack.accepted, ack.note


def test_the_same_order_twice_is_placed_once(fleet):
    """`order_id` is the idempotency key, and it is the only thing that makes a
    retry after a timeout safe. Without it the control plane's own retry loop
    would send two robots for one box."""
    leader = fleet.bot(2)
    fleet.link_all()
    leader.become_leader()

    stub = _stub(leader)
    first = stub.DispatchOrder(_order("order-3", 3, 9), timeout=5,
                              metadata=md(999, 0, "orders"))
    second = stub.DispatchOrder(_order("order-3", 3, 9), timeout=5,
                               metadata=md(999, 0, "orders"))

    assert first.accepted and second.accepted
    assert len([j for j in leader.jobs.all() if j.job_id == "order-3"]) == 1


def test_an_order_with_no_id_is_refused_and_says_why(fleet):
    """Not an error to raise on: the control plane retries, and retrying
    something that can never be idempotent would place it repeatedly."""
    leader = fleet.bot(2)
    leader.become_leader()

    ack = _stub(leader).DispatchOrder(
        _order("", 3, 9), timeout=5, metadata=md(999, 0, "orders"))

    assert not ack.accepted
    assert "order_id" in ack.note
