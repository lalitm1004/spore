# Why the fleet is decentralised

Two branches of this repository answered the same question in opposite ways, and
if you meet them cold the diff will not tell you which is right or why. This
records the argument while both sides are still legible.

The question is: **where does the fleet's knowledge live?**

---

## The two answers

**One service for the fleet.** `origin/webots-implementation` runs a single
`temp_network_interface` process. It holds the authoritative world state,
reconciles every outstanding command against what robots report, and journals
both so a restart resumes rather than forgets. Each robot container runs a
firmware process and a companion, and the companion dials that one service. Its
own comment puts the case plainly:

> One network layer for the whole fleet, reached over gRPC — not one process per
> robot. It holds the global fleet state, reconciles outstanding commands
> against what robots report, and journals both, none of which is possible from
> inside a single robot's process.

**One bot per robot.** This branch runs a `bot.py` beside every robot. They
elect a region leader by bully election, keep a roster by heartbeat, hand out
jobs from whichever leader owns the region, and reserve nodes *bot to bot*.
There is no process that knows everything, and that is the design rather than a
gap in it.

Both are coherent. They are not compatible, and the second is what this
repository builds.

---

## The claim in that comment is true

It is worth conceding properly, because a bad reason for the right answer is
worse than no reason. A per-robot process genuinely cannot hold global fleet
state. It sees its own region's roster and the leaders of other regions, and
nothing else. It cannot answer "where is every robot right now", it cannot
journal the fleet's history, and it cannot make a decision that requires knowing
what a robot three regions away is doing.

The disagreement is not about whether that is true. It is about whether it is a
loss.

---

## Why we accept it

**A warehouse robot's decisions are local.** Every question `bot.py` actually
answers — which way do I turn, is the node ahead taken, should I give way, am I
the nearest free bot to this pickup — needs its own position, its neighbours'
positions, and the map. None of it needs a fleet-wide picture. The one thing
that does, dispatching across regions, is handled by leaders talking to leaders,
which is a much smaller thing to keep correct than a shared world model.

**The single service is a single point of failure, and this floor cannot afford
one.** If it stops, every robot stops — not gracefully, but standing in lanes
holding nothing, because nothing else knows what was reserved. The decentralised
version has no such process. `PROTOCOL.md` §7 writes the rule as two things a
leader must *not* do:

> - Hold the only copy of any state — every bot caches the roster independently
> - Be required for safety-critical paths — reservations, collision avoidance,
>   energy shutoff work without the leader

A central service is both of those by construction. Reservations go bot to bot,
so a leader dying costs the fleet its dispatcher and its roster and costs it
nothing at all in not driving into each other. That is guarantee 4 in
`docs/scenarios.md`, and D8 is the scenario: containers, leader killed, claims
still crossing.

**Consistency is not free, and here it is not even wanted.** A central authority
has to be told where every robot is before it can answer anything, so every
decision waits on a round trip and on the freshness of a model that is always
slightly behind. Two robots meeting in a corridor need an answer in the time it
takes to stop. They already have what they need: each other's declared claims,
and a rule both sides compute identically. Neither has to ask.

**The demo is a decentralised protocol.** Killing one robot's coordinator is a
real thing to show, and it is only real if there is one to kill.

---

## What we give up, and what replaces it

| lost with no central service | what stands in |
|---|---|
| a fleet-wide view of every robot | leader-to-leader `BotLocation` exchange — every leader holds recent movement for its own region and hears the others' (`PROTOCOL.md` §3.2) |
| a durable journal of what happened | nothing. There is no fleet history today; if one is wanted it belongs in an observer that *reads* heartbeats, not in the path that answers robots |
| goals decided from a global optimum | leaders dispatch within a region and forward across; the assignment is good, not optimal, and it degrades to priority order rather than failing |
| one place to look when debugging | `AdminService.GetState` on any bot, which returns that bot's whole picture including its ledger |

The middle two are real costs. They are the price of the guarantee in §7, and
that trade is the decision this document exists to record.

---

## What we took from the other branch anyway

Rejecting the architecture is not rejecting the work. From
`webots-implementation` this branch takes the simulator and its world, the
headless recording path, the lane geometry, the GPU and stream-mode fix, and —
most importantly — **`network.proto`**, the shared JSON schemas rendered field
for field as protobuf. That file is the robot↔network contract and it was right;
what changed is only who serves it. Each bot serves it for its own robot instead
of one process serving it for all of them.

The one thing deliberately not taken is `temp_network_interface` itself, along
with its `Fleet`, `Journal`, `Relay` and `goal_policy`. Those are the central
authority, and they are the part that cannot be true at the same time as §7.

---

## If you are here to change this

Reversing the decision is legitimate, and it is not a refactor. It would delete
election (`election/`), the roster (`peers/`), bot-to-bot reservations
(`reservations/`), migration and leader dispatch (`bus/`), and roughly sections
4 through 15 of `PROTOCOL.md`. The planner (`planning/`) would survive, rehosted
centrally, but its traffic model would need rebuilding: tiers 1 and 2 are
*peers' declared claims and observed trails*, and a central service has no peers
— it has rows.

Do that on purpose or not at all.
