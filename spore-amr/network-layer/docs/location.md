# Where a robot is — the canonical location interface

Everything the fleet does is downstream of one number: which node each robot is
standing on. This is where that number comes from, where it goes, and what
quietly stops working when it is wrong.

It is worth being blunt about why this document exists. For the whole life of
this fleet, **every bot believed it stood at node 0, for ever**. The robots
drove — they were being told which way to turn at every marker — but the
coordination layer above them was running blind about itself, and because `0`
looks like a perfectly legal node, nothing failed loudly enough to notice.

---

## The path

```
  a QR tile under the camera
        │  cv2.QRCodeDetector                    webots/robot/qr.py
        ▼
  Read{node_id, region_id, kind, x_cm, y_cm}
        │  one report per tile crossing          webots/robot/main.py  Optics
        ▼
  EVT MARKER over the serial pty
        │                                        webots/robot/companion.py
        ▼
  RobotToNetwork{latest_node_id, region_id,
                 telemetry, mission, fault}      webots/robot/uplink.py
        │  gRPC, one long-lived stream
        ▼
  RobotNetworkServicer                           planning/robot_service.py
        │  Bot.report_robot_state
        ▼
  LatestRobotState  (a slot, not a queue)        bot.py
        │  read once per tick by the run loop
        ▼
  Bot.latest_node_id  +  Bot.node_trail          bot.py  _tick_robot_state
        │
        ├──► Heartbeat ──► the leader ──► roster ──► every bot in the region
        ├──► reservations: the node this robot is holding
        ├──► jobs: how far this bot is from a pickup
        ├──► migration: a QR in another region moves it
        └──► planning: tier 2, where the neighbours are heading
```

Two things about the shape.

**The QR read was always real.** `qr.py` has run a genuine `cv2.QRCodeDetector`
against a downward camera from the beginning, and the decoded node reached the
companion. What did not exist was the step after it: the companion asked which
way to turn and never said where it was. The reading was thrown away for every
purpose except the immediate turn.

**One message does both jobs.** A `RobotToNetwork` carrying `available` — the
nodes this robot can legally reach — is a question, and is answered. One without
it is telemetry, and is not. Both update position. That is the point: the fleet
learns where its robots are from the same messages that ask it where to send
them, so a robot that is driving is a robot that is reporting, with nothing extra
to remember to do.

---

## Why it is a node, and not a pose

The robot knows its continuous position better than this — it fuses odometry
with the marker fix and holds a pose in metres. None of that crosses the wire.

A node is what every other part of the fleet actually reasons about. Reservations
are on nodes. Routes are sequences of nodes. Regions are sets of nodes. A pose
would have to be turned back into a node by whoever received it, using a map they
would then need, and two receivers could round it differently. The robot is the
one standing on the tile; it reads the id off the floor, and that is not an
estimate.

The same argument, in the other direction, is why an answer names a node and
never a turn. See `PROTOCOL.md` §16.

---

## `node_trail`, and why the fleet cares which way you are facing

`latest_node_id` says where. `node_trail` — the last `NODE_TRAIL_LEN` (3)
*distinct* nodes, newest first, `node_trail[0] == latest_node_id` — says which
way. Two points give a heading, and on this floor plan a heading usually
*determines* the next hop rather than hinting at it: measured over all 1,904
directed steps of the real map, 64% have exactly one way on.

The trail is derived here, not sent. `_tick_robot_state` appends only on
movement — the same node reported twice is a robot standing still, not a path —
and the leader redistributes it in every `PeerRecord`. That is what tier 2 of the
traffic model extrapolates from when a neighbour has declared no claims:
`planning/traffic.py`.

---

## What breaks when it is wrong, and where

This is the table that makes the case. Every one of these failed silently for
the entire time position never arrived, because `0` is a legal-looking node and
nothing asserts otherwise.

| what stops working | where | how it looks |
|---|---|---|
| the bot never claims a node — `if not bot.latest_node_id: withdraw()` | `reservations/sender.py:89` | collision avoidance is inert; no bot holds anything |
| it is filtered out of every peer's announce list | `reservations/sender.py:151` | its neighbours never hear from it either |
| stall detection is permanently disabled | `bot.py:772` | a wedged robot is never escalated |
| dispatch scores every bot from node 0 | `bus/jobs.py:286` | "nearest free bot" silently becomes priority-and-id order |
| migration never fires — `desired_region_id` stays unset | `bus/migration.py:104` | a robot that drives into another region stays on the wrong leader's roster |
| tier 2 has no trail to extrapolate | `planning/traffic.py:107` | the planner treats every unannounced neighbour as stationary |

None of these raises. That is the whole problem with a location interface that
can be absent: the fleet keeps running, and keeps being wrong.

---

## A slot, not a queue

`LatestRobotState` holds the newest report and nothing else. The run loop reads
one item per tick (`T_HB`, a second) while a streaming robot reports every marker
and every status beat, so a FIFO would grow without bound and feed the loop
positions the robot left minutes ago — a bot steadily more confident about
somewhere its robot no longer is.

The consequence worth knowing: **a level survives collapsing and an edge does
not.** A robot reports "carrying, at the dropoff" on every tick until it is not,
so the fleet sees it however often it happens to read. Anything true for exactly
one report would be missed — which is why nothing on this wire is shaped that
way, and why a test that sent two reports a millisecond apart was wrong about
robots rather than finding a bug.

---

## Plugging in real hardware

The seam is `RobotSource` (`bot.py`), and it has not changed: implement `poll()`
returning the newest `RobotState` or `None`. What changed is that there is now a
real implementation to copy, and a wire that a Raspberry Pi speaks exactly as the
simulator does — `proto/robot.proto`, rendered from the shared JSON schemas in
`spore-amr/shared/schemas/` field for field.

A real robot differs from the simulated one in what reads the QR code and what
turns the wheels. It does not differ in what it says.

---

## What is still not carried

Being honest about the edges of this interface:

- **No pose.** By design, above — but it does mean the fleet cannot tell a robot
  halfway along a lane from one sitting on the node behind it. `PROTOCOL.md`
  §14.6 records the one place that shows: job distance is measured from the last
  scanned node.
- **No FSM state.** The wire has no field for MOVING versus IDLE. A robot
  reporting at a node is standing at it, and what it does between nodes is not
  something this link describes.
- **Battery only.** `Telemetry` carries a percentage and nothing else. Motor
  temperature, wheel slip and the rest have no field yet, and should get one
  here rather than in a fault string.
- **Faults are typed going up, flat going across.** `Fault.error.type` is an
  enum on the robot link; `PeerRecord.fault` is still a string, because the
  leader displays it and nothing parses it.
