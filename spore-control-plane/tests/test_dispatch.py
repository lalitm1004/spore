"""Control plane behaviour: map validation, dispatch + retry, and the web layer.

The fleet's own `ControlPlaneService` implementation doesn't exist yet, so
everything here runs against the mock server from `tests.conftest`.
"""
from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from spore_control_plane import config
from spore_control_plane.app import create_app
from spore_control_plane.map import NullMap, WarehouseMap
from spore_control_plane.proto import controlplane_pb2
from spore_control_plane.submitter import DispatchError, OrderSubmitter


# ---------------------------------------------------------------- map

def test_map_knows_which_nodes_exist():
    m = WarehouseMap.load(config.WAREHOUSE_MAP)
    assert isinstance(m, WarehouseMap)
    assert m.has_node(0) and m.has_node(1)
    assert not m.has_node(10**9)


def test_null_map_accepts_every_node():
    m = NullMap()
    assert m.has_node(123)


# ---------------------------------------------------------------- dispatch

def _order() -> controlplane_pb2.Order:
    return controlplane_pb2.Order(order_id="cargo-1", pickup_node=10, dropoff_node=20)


def test_dispatch_to_a_bot(mock_server):
    addr, mock = mock_server
    ack = OrderSubmitter([addr]).dispatch(_order())
    assert ack.accepted and ack.owner_region == 14
    assert mock.orders[0].order_id == "cargo-1"


def test_dispatch_falls_back_to_another_bot(mock_server):
    good_addr, mock = mock_server
    submitter = OrderSubmitter(["127.0.0.1:1", good_addr])
    ack = submitter.dispatch(_order())
    assert ack.accepted, "a dead first bot must not stop dispatch"
    assert mock.orders[0].order_id == "cargo-1"


def test_dispatch_retries_then_succeeds(mock_server, monkeypatch):
    # One bot that is briefly unreachable mid-election; a later attempt lands.
    addr, mock = mock_server
    attempts = {"n": 0}

    def flaky_call(self, address, order):
        attempts["n"] += 1
        if attempts["n"] == 1:
            return None
        return mock.ack

    monkeypatch.setattr(config, "DISPATCH_ATTEMPTS", 3)
    monkeypatch.setattr(config, "DISPATCH_BACKOFF", 0.0)
    monkeypatch.setattr(OrderSubmitter, "_call", flaky_call)

    ack = OrderSubmitter([addr]).dispatch(_order())
    assert ack.accepted
    assert attempts["n"] >= 2


def test_dispatch_raises_after_all_attempts(mock_server, monkeypatch):
    addr, mock = mock_server
    mock.ack = controlplane_pb2.DispatchAck(accepted=False, note="nobody free")
    monkeypatch.setattr(config, "DISPATCH_ATTEMPTS", 2)
    monkeypatch.setattr(config, "DISPATCH_BACKOFF", 0.0)

    with pytest.raises(DispatchError):
        OrderSubmitter([addr]).dispatch(_order())


# ---------------------------------------------------------------- web layer

def test_web_creates_order_end_to_end(mock_server):
    addr, mock = mock_server
    m = WarehouseMap.load(config.WAREHOUSE_MAP)
    pickup = 0
    dropoff = 1
    assert m.has_node(pickup) and m.has_node(dropoff)

    app = create_app(warehouse_map=m, submitter=OrderSubmitter([addr]))
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
    app = create_app(warehouse_map=m, submitter=OrderSubmitter([addr]))
    client = TestClient(app)

    resp = client.post("/orders", data={"pickup_node": "999999", "dropoff_node": "1"})
    assert resp.status_code == 200
    assert "not a known node" in resp.text
