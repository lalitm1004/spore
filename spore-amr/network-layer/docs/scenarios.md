# What the fleet does — the behavioural contract

This is the fleet's answer to "what happens if…", one situation at a time, with
the test that proves each answer beside it. Every scenario here runs on real
containers on a real Docker network: separate processes, a network that can be
partitioned, and processes that can be killed.

It is written to be read by someone who has not seen the code. `PROTOCOL.md` is
the design; this is the behaviour.

Run them: `uv run pytest tests -q -m docker`. Ids below match test names, so
`-k A1` runs exactly the scenario you are reading.

---

## The four guarantees

Everything else is detail. These four are the things the fleet must not do,
whatever else goes wrong.

**1. A robot is never answered with silence.** It stops at a QR node, asks which
way to turn, and blocks. If it hears nothing it stays there for the rest of its
shift, because it only asks again on reaching the next node — and it will not
reach one. So a malformed question, a planner that raised, a map that disagrees
with the robot's, a goal it cannot reach: all of them get an answer. A wrong
turn is recoverable at the next node. Silence is not recoverable at all.

**2. No two robots ever hold one node at the same time.** Not "rarely" — never.
Reservations address nodes only, and a robot holds *both* endpoints for the
whole of a traversal, so two robots swapping across an edge both claim both ends
and the clash surfaces as an ordinary overlap. `assert_no_overlap` re-checks
this wherever bots share a floor.

**3. Both sides of a contest reach the same verdict alone.** When two robots
want the same node they do not negotiate. Each applies the same ordering —
carrying cargo beats heading for a pickup beats free, ties on the lower bot id —
to the same two numbers, which both already have. They cannot both give way and
cannot both stay.

**4. Collision avoidance survives the leader.** Reservations travel bot to bot,
never through a region leader, because `PROTOCOL.md` §7 requires exactly this.
Kill the leader and claims keep flowing.

---

## A. Decisions — answering the robot

The robot resolves which turns physically exist from its own copy of the map and
asks which to take. We answer with one of the ones it offered.

| id | situation | what happens |
|---|---|---|
| A1 | it has no job | `WAIT` with a hold, so it asks again shortly rather than spinning |
| A2 | it has a job | `PROCEED`, naming a lane the robot said exists |
| A3 | it is standing on its goal | `WAIT` — there is nowhere to be until a new job arrives |
| A4 | nothing about the route changed | `PROCEED`, not `REROUTE`. The two differ only in whether the route moved, which is what makes a log readable when a bot doubles back |
| A5 | the turns offered exclude the one we planned | still answered, with a lane it did offer |
| A6 | a malformed line arrives | logged and skipped; the next question is answered on the same connection |
| A7 | it is standing on a node our map has never heard of | still answered |
| A8 | the bot booted with no map file | `WAIT` saying so — and it still elects, heartbeats and holds a region. Geography-blind is a degraded fleet, not a dead one |
| A9 | two junctions share a destination | the `query_id` comes back exactly, which is the only way the robot tells a fresh answer from a late one |
| A10 | a whole shift of questions | one connection throughout. A socket per question is overhead these bots cannot spare |

## B. Job distribution

A job names a pickup and a dropoff. Which robot goes is the leader's call, made
from the roster, which is only as fresh as the last heartbeat.

| id | situation | what happens |
|---|---|---|
| B1 | two free bots, one nearer | the nearer one goes — by driving distance, not by how the nodes are numbered |
| B2 | the nearer one is low on charge | the charged one goes. A bot that needs charging mid-job is the wrong bot |
| B3 | a bot already has a job | it is not given a second |
| B4 | a bot is below `JOB_MIN_BATTERY` | never assigned |
| B5 | a bot reports a fault | never assigned |
| B6 | only the leader is free | the leader takes it, and observes its own progress. Last resort: a leader that drives away has to hand off first |
| B7 | bot-0 is the only free bot | assigned. Its id is falsy, and an `if assignee:` once made the first bot quietly unassignable |
| B8 | the same job is submitted twice | assigned once. `job_id` is the cargo id, so a repeat is the same cargo |
| B9 | a job is handed to a follower | routed to its leader, which assigns |
| B10 | the pickup is in another region | forwarded to that region's leader, which owns it from then on |
| B11 | nobody anywhere can take it | accepted and queued, then assigned when a bot frees up. Refusing would lose the cargo |
| B12 | the job runs to completion | `PICKUP → EN_ROUTE → DROPOFF → DELIVERED`, crossed off the ledger, bot free |

## C. Planning

| id | situation | what happens |
|---|---|---|
| C1 | a job is assigned | it becomes a turn at every node, not one command naming somewhere seventy hops away |
| C2 | the cargo is picked up | the goal becomes the dropoff. Nobody re-commands the robot; collecting the cargo is what changes where it is going |
| C3 | a neighbour has claimed a node on our route | we do not drive into it |
| C4 | a neighbour is moving but has not claimed anything | its trail reaches our roster, which is all prediction has to work from |
| C6 | our battery is nearly flat | waiting beats going round. The energy term exists to make exactly this trade |
| C7 | a full roster and a long route | answered in single-digit milliseconds, against a one-second tick |
| C8 | the goal cannot be reached | said out loud, with a reason. "I cannot get there" has to be spoken |

## D. Exceptions

| id | situation | what happens |
|---|---|---|
| D1 | commanded, but not moving for one `T_STALL` | the route is dropped and replanned. Cheapest suspicion first |
| D2 | still not moving after two | claims released — it stops holding a lane it is not using |
| D3 | still not moving after three | escalated to the control plane. A person should look |
| D4 | a fault before the cargo is collected | the job goes back. Otherwise it would resume after recovering and two bots would go for one crate |
| D5 | a fault after the cargo is aboard | the job stays, and is escalated. The cargo is physically on this robot; dropping the job would lose track of where it is |
| D6 | the companion goes away | not an error — a shift ending. The next one connects to the same listener |
| D7 | a bot dies holding a lane | its claims lapse and the lane opens. One crash must not close a corridor for the rest of the shift |
| D8 | the leader dies | claims keep flowing — guarantee 4 |

## E. Collisions

| id | situation | what happens |
|---|---|---|
| E1 | two bots claim one node | exactly one ends up holding it |
| E2 | three bots claim one node | still exactly one. The ordering is total, so a third claimant changes nothing |
| E4 | one bot follows another closely | the follower does not close up. A node is held until the robot is fully inside the next one, so arriving early is not possible |
| E5 | two bots in claim range | a claim crosses the network into the other's ledger |
| E6 | any of the above | **no node was ever held by two bots at once** — guarantee 2, re-checked wherever bots share a floor |

## F. Redirections

| id | situation | what happens |
|---|---|---|
| F2 | a node is reported blocked | the route goes around it |
| F3 | the blockage clears | the lane is usable again. A blockage that is gone must stop costing something, or the fleet slowly forgets lanes it can use |
| F4 | a bot migrates mid-route | it replans on arrival and does not go quiet. Regions are subnetworks: a route across one is planned optimistically and only becomes informed once the new roster lands |
| F6 | a peer claims our next node between two questions | the second answer names a different lane. Traffic is not static between one node and the next |

## G. Yielding

Giving way is the only option that means leaving the route, so it needs a rule
rather than falling out of the search. We yield when the wait is long, we *lose*
the priority comparison, and there is somewhere to stand aside. When we win it,
we hold — the other robot is the one that should move.

| id | situation | what happens |
|---|---|---|
| G1 | they carry cargo, we are free | we give way. A robot with cargo aboard is not asked to reverse out of a corridor for an idle one |
| G2 | both free, we have the lower id | we hold. The tiebreak is ours |
| G7 | a symmetric standoff | exactly one gives way. Never both, never neither — guarantee 3 |

## H. Membership

Bootstrap and convergence, a killed leader and its restart, a hung leader and
the split brain healing, a partitioned follower self-electing and yielding on
reconnect, two regions meeting, and a job crossing between them. These predate
the scenarios above and are unchanged.

---

## What is not covered, and why

Being straight about the holes is the point of writing this down.

- **F1, promoting a cached alternate, has no test because the fleet does not
  work that way.** Holding alternatives pays off when losing a route means
  paying for a fresh search. The robot asks at every node and a plan costs under
  a millisecond, so the answer is recomputed each time and an alternative in hand
  would save nothing. `planning/routes.py` stays, unwired and unit-tested, for a
  model that commits to longer routes.
- **The yield-spot cascade (a `YI` bay, else a junction, else a bay) is
  unit-tested, not container-tested.** Which spot is chosen depends on the map
  around the contested corridor, and pinning that in a container test would be
  asserting the map rather than the rule.
- **E3, two bots driving head-on through a whole corridor**, is not written.
  E4, G1 and G7 cover the contest itself at the point it matters.
- **The first decision after migrating is uninformed.** A follower's roster
  covers its own region only, so a route into another one is planned
  optimistically. Accepted, not solved.
- **No scenario here runs a camera.** Every robot in this tier speaks the real
  wire and reports honestly, but none of them reads a QR code: the `cv2` decode
  and the physics belong to the Webots tier, which is slower and separate. What
  that leaves untested here is the step *before* a report — whether the node in
  it was read correctly — and nothing else.

## What compressed timings do and do not prove

These run on a fast clock — `T_HB` at 0.3 s instead of 1 s, and the timeouts
that derive from it shortening with it — because fifty-odd scenarios at
production timings would take too long to run often enough to be worth having.

That means they prove the *logic* holds, not that the production timings are
right. A scenario that only passes on the fast clock belongs in a
production-timing tier, not in a version of itself tuned until it passes. All
timings come from one `FAST_TIMINGS` dict in `tests/test_docker.py` so no
scenario can quietly invent its own.

The shared fleets take one further change, `SHARED_TIMINGS`, which lengthens the
stall clock. A bot on a shared fleet is put where a scenario wants it and then
stands there while the scenario asks its questions, and the fleet is right to
call that a stall — but it is a fact about a fleet nobody is driving, not about
anything under test. D1–D3, which *are* about stalling, launch their own fleet
on the short clock.

## How a robot is put somewhere

Every scenario here places a robot by **telling the truth about where it is**,
over `RobotNetwork.Session` — the same stream a companion speaks, and the only
thing one can say.

It was not always so. These scenarios used to use an admin RPC that pushed a
whole robot snapshot straight into the bot, around the QR read, the companion
and the wire. That cost more than it looked: injection supplied by hand the one
thing production never supplied, so no test could see that the fleet had never
learned a single robot's position. It also let a scenario put two robots on one
node, which driving cannot do — and the invariant that forbids it then failed,
correctly, on a state the harness had invented.

Two consequences, both load-bearing. A robot moves one hop per `_hop_seconds()`,
derived from the same kinematics the claim window is, because a harness that
moves robots faster than robots move makes every claim overlap. And an
obstruction is *reported* rather than pushed: a robot says "I am here and
something is in front of me", and the network layer blocks the lane it last sent
that robot down, because it is the one that knows which lane that was.
