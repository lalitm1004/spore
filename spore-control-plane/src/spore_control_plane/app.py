"""The web layer: an order form that hands orders to the fleet over gRPC.

WHAT
    * `GET /`        — render the order form.
    * `POST /orders` — validate the pickup/dropoff nodes, mint an order id,
                       and dispatch the order to any reachable bot (the fleet
                       routes it to the right region/leader).

WHERE
    Entry point is `spore_control_plane.__init__.main`, which builds the app
    with production singletons and serves it with uvicorn.

WHY
    The control plane exists so an operator can create an order without knowing
    anything about the fleet — which robot leads, where the cargo starts, how
    jobs are tracked. This layer turns two node ids into a dispatched order and
    shows the result.

HOW
    A `create_app` factory takes an optional `WarehouseMap` and `OrderSubmitter`
    (defaults are built from `config`), so tests can inject fakes and the
    production entry point stays a one-liner.
"""
from __future__ import annotations

import logging
import time
import uuid
from pathlib import Path

from fastapi import FastAPI, Form, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates

from spore_control_plane import config
from spore_control_plane.map import WarehouseMap
from spore_control_plane.proto import controlplane_pb2
from spore_control_plane.submitter import DispatchError, OrderSubmitter

log = logging.getLogger(__name__)

TEMPLATES_DIR = Path(__file__).resolve().parent.parent.parent / "templates"


def create_app(
    warehouse_map: WarehouseMap | None = None,
    submitter: OrderSubmitter | None = None,
) -> FastAPI:
    app = FastAPI(title="Spore Control Plane")
    templates = Jinja2Templates(directory=str(TEMPLATES_DIR))

    # Built once at startup; the caller may inject fakes for tests.
    wh_map = warehouse_map if warehouse_map is not None else WarehouseMap.load(config.WAREHOUSE_MAP)
    sub_ = submitter if submitter is not None else OrderSubmitter()

    @app.get("/", response_class=HTMLResponse)
    async def index(request: Request):
        return templates.TemplateResponse(request, "order.html", {"result": None})

    @app.post("/orders", response_class=HTMLResponse)
    async def create_order(
        request: Request,
        pickup_node: str = Form(...),
        dropoff_node: str = Form(...),
        order_id: str = Form(default=""),
    ):
        errors = _validate(pickup_node, dropoff_node, wh_map)
        if errors:
            return templates.TemplateResponse(request, "order.html", {"result": {"error": errors[0]}})

        pickup = int(pickup_node)
        dropoff = int(dropoff_node)
        oid = order_id.strip() or str(uuid.uuid4())

        order = controlplane_pb2.Order(
            order_id=oid,
            pickup_node=pickup,
            dropoff_node=dropoff,
            timestamp=int(time.time() * 1000),
        )

        try:
            ack = sub_.dispatch(order)
        except DispatchError as e:
            log.error("dispatch failed: %s", e)
            return templates.TemplateResponse(request, "order.html", {
                "result": {"error": f"dispatch failed: {e}"},
            })

        result = {
            "order_id": oid,
            "pickup_node": pickup,
            "dropoff_node": dropoff,
            "accepted": ack.accepted,
            "owner_region": ack.owner_region,
            "assignee": ack.assignee if ack.HasField("assignee") else None,
            "note": ack.note,
        }
        return templates.TemplateResponse(request, "order.html", {"result": result})

    return app


def _validate(pickup_node: str, dropoff_node: str, wh_map: WarehouseMap) -> list[str]:
    errors: list[str] = []
    for label, raw in (("pickup_node", pickup_node), ("dropoff_node", dropoff_node)):
        try:
            node_id = int(raw)
        except ValueError:
            errors.append(f"{label} must be an integer")
            continue
        if node_id < 0:
            errors.append(f"{label} must be >= 0")
            continue
        if not wh_map.has_node(node_id):
            errors.append(f"{label} {node_id} is not a known node in the warehouse map")
    return errors
