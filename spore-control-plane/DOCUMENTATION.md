# spore-control-plane — Design Documentation

## Goal

Provide a single, central service where an operator (or an external order
system) can create cargo orders — *"collect goods at node-X, deliver them to
node-Y"* — and have each order dispatched, over gRPC, to the leader of the
warehouse region where the order starts.

The project is also the Kubernetes control plane that boots the AMR fleet and
the Webots simulator, so it already knows every robot's address. It does **not**
know which robot leads which region, and it is deliberately **not** part of the
fleet's membership protocol.

## Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | **Separate proto, owned by this project** (`controlplane.proto`) | The dependency is one-way: the network layer implements *our* contract, not the other way around. We never import or depend on `fleet.proto`. |
| 2 | **One `ControlPlaneService`, two RPCs** (`DispatchOrder`, `DiscoverLeaders`) | A single service means the network layer adds one registration and one policy entry. |
| 3 | **Dispatch to the pickup-region leader, fall back to any bot** | The user requirement is to hit the leader where the order starts; when discovery is stale/incomplete, any bot will forward to the right leader anyway. |
| 4 | **Orders are fire-and-forget** | We submit and report the ack. Delivery/failure feedback (e.g. `NEEDS_ATTENTION`) is out of scope for v1 and can be added later as a streaming RPC on the same service. |
| 5 | **`order_id` is a UUID minted here (== the fleet's `cargo_id`)** | Orders come from an external system, so IDs cannot come from a central sequential allocator. The UUID doubles as the idempotency key: retries are safe. |
| 6 | **Bots do the routing; we only choose a target bot** | A follower forwards a dispatch to its leader; a leader forwards to the pickup region's leader. This logic already exists in the fleet, so our client stays a thin caller. |
| 7 | **Reserved control-plane identity in metadata** | The fleet's virtual network requires `bot-id` / `region-id` / `role` metadata on every call. We use a reserved `bot-id` so the fleet can later admit us via its policy table without confusing us for a robot. |
| 8 | **Python + FastAPI + grpcio** | Matches the rest of the Spore stack; FastAPI gives a minimal web UI with little ceremony. |

## What we do

```
operator ──► [web UI] ──► POST /orders (pickup_node, dropoff_node)
                              │
                              ├─ validate nodes against warehouse-layout.json
                              ├─ resolve pickup_node → region_id
                              ├─ mint order_id = uuid4()
                              │
                              ├─ DiscoverLeaders ──► cache region → leader
                              └─ DispatchOrder ────► leader of pickup region
                                                       (fallback: any bot)
```

1. **Web UI** — a form to create an order (pickup node, dropoff node, optional
   order id).
2. **Validation + region lookup** — load `warehouse-layout.json` (the same map
   the fleet uses) to confirm the node ids exist and to find the pickup
   region. A missing map degrades gracefully: skip validation, still dispatch.
3. **Order → gRPC** — build an `Order`, then call `DispatchOrder` on the target
   leader. Retry the *same* `order_id` across known bots on transient failure
   (idempotent).
4. **Leader discovery** — maintain a `region → leader` cache refreshed via
   `DiscoverLeaders`, so dispatch is normally direct.

## Why we do it this way

- **No coupling to the fleet's internals.** The network layer only has to
  implement `controlplane.proto`. Our knowledge of it (job ids, forwarding,
  region semantics) is encoded in the contract, not in shared code.
- **Resilient to leadership changes.** We never hard-code leaders; discovery is
  live and dispatch falls back to any bot, whose forwarding guarantees delivery
  to the right region.
- **Idempotent and retryable.** The UUID `order_id` means a double-submit or a
  retry after a timeout cannot create a duplicate job in the fleet.
- **Minimal surface.** One service, two RPCs, one web endpoint — easy for the
  network layer to implement and easy to reason about.

## Contract (`proto/controlplane.proto`)

See the file itself; the key messages are:

- `Order { order_id, pickup_node, dropoff_node, timestamp }`
- `DispatchAck { accepted, owner_region, assignee?, note }`
- `LeaderInfo { region_id, bot_id, address }`
- `DiscoverLeadersResponse { leaders[], self_region_id, self_leader_bot_id, self_leader_address }`

### What the network layer must implement (later)

1. Copy `controlplane.proto` and regenerate its stubs.
2. `DispatchOrder` → map `Order` to the fleet's internal job and call the
   existing dispatcher (a thin adapter — the routing already exists).
3. `DiscoverLeaders` → return the bot's known leaders + its own region/leader.
4. Serve `ControlPlaneService` on **every** bot, and add a policy entry that
   admits our reserved identity.
