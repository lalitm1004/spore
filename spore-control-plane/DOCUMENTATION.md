# spore-control-plane — Design & Integration Documentation

## Goal

Provide a single, central service where an operator (or an external order
system) can create cargo orders — *"collect goods at node-X, deliver them to
node-Y"* — and have each order handed to the fleet over gRPC, which routes it
to the leader of the region where the order starts.

The project is also the Kubernetes control plane that boots the AMR fleet and
the Webots simulator, so it already knows every robot's address. It knows
**nothing else** about the fleet — not regions, not leaders, not which bot is
where — and it is deliberately **not** part of the fleet's membership protocol.

---

## The one rule: the control plane knows no geography

The fleet is dynamic: bots migrate between regions, leaders rotate by tenure,
fault out, die and are re-elected. It is therefore **impossible** for an outside
service to know which region a bot is in, or who leads what, at any given
moment. Only the fleet's live membership knows.

So the control plane does the only thing it can correctly do: it hands an order
to **any reachable bot** and lets the fleet route it. The fleet already does the
routing internally — a non-leader forwards to its leader; a leader resolves
`pickup_node`'s region from its own map and forwards to that region's leader
(`network-layer/bus/jobs.py: Dispatcher.submit`). The control plane never
computes a region, never caches a leader, never targets a bot by location.

The one thing it does know — the list of bot addresses — is stable and comes
from the fact that it boots the fleet.

```
order (pickup node X, dropoff node Y)
  └─ mint order_id = uuid4()
  └─ for attempt in 1..M:
       for each known bot address:
         DispatchOrder(order -> that bot)      # bot forwards as needed
         if ack.accepted: done
       backoff                                  # election / migration in flight
  └─ error after M attempts
```

Retries reuse the *same* `order_id`, which is the fleet's idempotency key
(`job_id == cargo_id == order_id`), so a retry after a timeout cannot
double-place the order.

---

## What it does today (tangible)

This is a complete, runnable service — not a stub. Concretely:

- **Serves a web UI** at `GET /` with an order form, and accepts orders at
  `POST /orders` (`pickup_node`, `dropoff_node`, optional `order_id`).
- **Validates node ids** against `warehouse-layout.json` (881 nodes): rejects
  non-integer, negative, or unknown node ids before anything leaves the
  process. (This is the *only* use of the map — validation, not routing.)
- **Mints the order id** — a UUID if the caller didn't supply one — which is the
  fleet's `cargo_id` and therefore the idempotency key.
- **Dispatches over gRPC** (`DispatchOrder`) to any known bot, retrying up to
  `DISPATCH_ATTEMPTS` passes over the address list with `DISPATCH_BACKOFF`
  between passes, so a leader election or a migration mid-flight doesn't lose
  the order.
- **Reports the outcome** — `owner_region`, immediate `assignee` (if any), and
  the fleet's `note` — back on the result page. (The region comes from the
  fleet's ack; the control plane never computes it.)
- **Degrades gracefully**: no map → no validation but dispatch still works; no
  bots → the app still serves and dispatch reports "not dispatched".

Verified by tests (`uv run pytest`) that exercise node validation, dispatch,
fallback across bots, retry, and the full web → dispatch path against a mock
`ControlPlaneService` server.

### File map (what lives where)

| Concern | File |
|---|---|
| The contract the fleet implements | `proto/controlplane.proto` |
| Generated stubs | `src/spore_control_plane/proto/controlplane_pb2*.py` |
| Env-driven configuration | `src/spore_control_plane/config.py` |
| Node-id validation | `src/spore_control_plane/map.py` |
| Order dispatch + retry loop | `src/spore_control_plane/submitter.py` |
| gRPC channel pool + wire identity | `src/spore_control_plane/client.py` |
| Web UI (`GET /`, `POST /orders`) | `src/spore_control_plane/app.py` |
| Order form template | `templates/order.html` |
| Map copy (self-contained) | `shared/warehouse-layout.json` |

---

## What it deliberately does not do

- **No region/leader knowledge.** It never resolves or stores regions, leaders,
  or bot locations; routing is the fleet's job.
- **No status feedback.** Dispatch is fire-and-forget: we report the ack and
  stop. Delivery / failure (`NEEDS_ATTENTION`) push-back can be added later as
  a streaming RPC on the same service.
- **No path planning or hop-distance math.** The fleet owns navigation.
- **No authentication.** The fleet's virtual network is isolation, not auth;
  we match that posture and just present a reserved identity.
- **No fleet membership.** We never heartbeat, never vote, never appear in a
  roster. Our `bot-id` is reserved (`CONTROL_BOT_ID=9000`, outside the fleet's
  `<100` space) precisely so we can never be mistaken for a robot.

---

## Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | **Separate proto, owned by this project** (`controlplane.proto`) | The dependency is one-way: the network layer implements *our* contract. We never import `fleet.proto`. |
| 2 | **One service, one RPC** (`ControlPlaneService.DispatchOrder`) | The absolute minimum surface the fleet must implement. |
| 3 | **Dispatch to any bot; the fleet routes** | Regions/leaders are unknowable from the outside and ephemeral; the fleet's dispatcher is the only authoritative router. |
| 4 | **Orders are fire-and-forget** | Minimal v1; status can be a later streaming RPC on the same service. |
| 5 | **`order_id` is a UUID minted here (== the fleet's `cargo_id`)** | External orders can't use a central allocator; the UUID doubles as the idempotency key. |
| 6 | **Retry M passes over the address list** | Rides out the transient window of a leader election or migration. Idempotent, so safe. |
| 7 | **Reserved control-plane identity in metadata** | The fleet's virtual network requires `bot-id`/`region-id`/`role` on every call; a reserved id lets it admit us without confusing us for a robot. |
| 8 | **Python + FastAPI + grpcio** | Matches the rest of the Spore stack; FastAPI gives a minimal web UI with little ceremony. |
| 9 | **Map and proto are copied, not linked** | Self-contained image; the control plane builds and runs with no dependency on `network-layer` or `spore-amr`. |

---

## The contract (`proto/controlplane.proto`)

Key messages (see the file for full comments):

- `Order { order_id, pickup_node, dropoff_node, timestamp }`
- `DispatchAck { accepted, owner_region, assignee?, note }`
- `service ControlPlaneService { rpc DispatchOrder(Order) returns (DispatchAck) }`

The wire identity we present on every call is:

```
bot-id = 9000   (reserved; configurable via CONTROL_BOT_ID)
region-id = 0   (configurable)
role = "control"
```

---

## Integration: done, and where it lives

This section used to be a copy-pasteable edit list for the network layer. The
edits are made, so the list is replaced by the map of where they landed:

| edit | where |
|---|---|
| the proto, vendored | `network-layer/proto/controlplane.proto` — byte-identical to this project's; `tests/test_control_plane.py` fails on drift |
| the servicer | `network-layer/bus/control_plane.py` — `Order` → `Job` → `Dispatcher.submit`; nothing else |
| registered on every bot | `bot.py:_start_grpc_server` |
| admitted by the virtual network | `bus/policy.py:_allowed`, alongside `JobService` |
| in the demo fleet | `webots/tools/gen_fleet.py` emits a `control` service with every bot's address; `fleet.sh order` places one |

The one design point worth keeping from the old list: `Order` maps onto the
dispatcher's existing entry, not onto a second path. Idempotency, forwarding,
queueing and retry all live in `bus/jobs.py`, and a second implementation of
any of them would be a second thing to keep right.

## Acceptance checklist for integration

1. `DispatchOrder` to a *follower* lands with its leader; to a *leader* of the
   wrong region lands with the pickup region's leader (observable via
   `owner_region` in the ack).
2. Re-submitting the same `order_id` returns the existing assignment, not a
   duplicate.
3. A call with no metadata is refused `UNAUTHENTICATED` (existing policy
   behaviour, unchanged).
4. `controlplane.ControlPlaneService` is accepted with the reserved identity and
   does not appear in any roster.

---

## Why this shape

- **No coupling to the fleet's internals.** The network layer only has to
  implement one RPC. Everything we know about it (job ids, forwarding) is
  encoded in the contract, not in shared code.
- **No geography to keep in sync.** The control plane holds no region or leader
  state, so there is nothing that can go stale when bots migrate or leaders
  rotate.
- **Idempotent and retryable.** The UUID `order_id` means a double-submit or a
  retry after a timeout cannot create a duplicate job in the fleet.
- **Minimal surface.** One service, one RPC, one web endpoint — easy for the
  network layer to implement and easy to reason about.
