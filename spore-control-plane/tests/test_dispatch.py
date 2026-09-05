"""Control plane behaviour: map, discovery, dispatch, and the web layer.

The fleet's own `ControlPlaneService` implementation doesn't exist yet, so
everything here runs against the mock server from `tests.conftest`.
"""
from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from spore_control_plane import config
from spore_control_plane.app import create_app
from spore_control_plane.discovery import LeaderDirectory
from spore_control_plane.map import NullMap, WarehouseMap
from spore_control_plane.proto import controlplane_pb2
from spore_control_plane.submitter import DispatchError, OrderSubmitter

# ---------------------------------------------------------------- map

def test_map_loads_and_resolves_regions():
    m = WarehouseMap.load(config.WAREHOUSE_MAP)
    assert isinstance(m, WarehouseMap)
    nodes = m.nodes_in(14)          # parking
    assert nodes, "the shared map should contain parking nodes"
    assert m.region_of(nodes[0]) == 14
    assert m.has_node(nodes[0]) and not m.has_node(10**9)


def test_null_map_degrades_gracefully():
    m = NullMap()
    assert m.has_node(123) and m.region_of(123) is None and m.nodes_in(1) == []


# ---------------------------------------------------------------- discovery

def test_discovery_merges_leaders_and_self(mock_server):
    addr, mock = mock_server
    mock.leaders = [controlplane_pb2.LeaderInfo(region_id=2, bot_id=8, address="bot-8:50051")]
    mock.self_region_id = 14
    mock.self_leader_bot_id = 0
    mock.self_leader_address = addr

    d = LeaderDirectory([addr])
    assert d.leader_for(2).bot_id == 8
    # The answering bot's own region/leader is folded in.
    assert d.leader_for(14).bot_id == 0
    assert mock.discovery_requests >= 1


# ---------------------------------------------------------------- dispatch

def _order() -> controlplane_pb2.Order:
    return controlplane_pb2.Order(order_id="cargo-1", pickup_node=10, dropoff_node=20)


def test_dispatch_to_leader(mock_server):
    addr, mock = mock_server
    ack = OrderSubmitter([addr]).dispatch(_order(), region_id=14, leader_address=addr)
    assert ack.accepted and ack.owner_region == 14
    assert mock.orders[0].order_id == "cargo-1"


def test_dispatch_falls_back_when_leader_unreachable(mock_server):
    addr, mock = mock_server
    ack = OrderSubmitter([addr]).dispatch(_order(), region_id=14, leader_address="127.0.0.1:1")
    assert ack.accepted, "a dead leader must not stop dispatch: any bot is tried next"
    assert mock.orders[0].order_id == "cargo-1"


def test_dispatch_raises_when_nobody_accepts(mock_server):
    addr, mock = mock_server
    mock.ack = controlplane_pb2.DispatchAck(accepted=False, note="nobody free")
    with pytest.raises(DispatchError):
        OrderSubmitter([addr]).dispatch(_order(), region_id=14, leader_address=addr)


# ---------------------------------------------------------------- web layer

def test_web_creates_order_end_to_end(mock_server):
    addr, mock = mock_server
    m = WarehouseMap.load(config.WAREHOUSE_MAP)
    pickup = m.nodes_in(14)[0]
    dropoff = m.nodes_in(14)[1]

    mock.self_region_id = 14
    mock.self_leader_bot_id = 0
    mock.self_leader_address = addr

    app = create_app(warehouse_map=m, directory=LeaderDirectory([addr]), submitter=OrderSubmitter([addr]))
    client = TestClient(app)

    resp = client.post("/orders", data={"pickup_node": str(pickup), "dropoff_node": str(dropoff)})
    assert resp.status_code == 200
    assert "Order dispatched" in resp.text

    assert len(mock.orders) == 1
    order = mock.orders[0]
    assert order.pickup_node == pickup and order.dropoff_node == dropoff
    assert order.order_id, "a UUID order id must be minted"


def test_web_rejects_unknown_node(mock_server):
    addr, _ = mock_server
    m = WarehouseMap.load(config.WAREHOUSE_MAP)
    app = create_app(warehouse_map=m, directory=LeaderDirectory([addr]), submitter=OrderSubmitter([addr]))
    client = TestClient(app)

    resp = client.post("/orders", data={"pickup_node": "999999", "dropoff_node": "1"})
    assert resp.status_code == 200
    assert "not a known node" in resp.text
