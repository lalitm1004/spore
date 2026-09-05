"""Many independent planners on one map, stepped together.

This is the end-to-end check the package rests on, and it needs no Webots. Each
robot gets its own `Planner`, sees only what a heartbeat would carry about the
others, and replans on its own schedule. What the run then asserts is the property
that actually matters: that robots which never coordinate directly still do not
collide.

Two modelling choices are worth being explicit about.

*There is no shared table.* Every robot keeps its own `p2p.Ledger` and learns what
its neighbours hold only from the announcements they send, to the handful of bots
within claim range. Nothing in this simulation has a god's-eye view of who holds
what -- which is the point, because the real fleet will not have one either. A
robot enters a node when its own ledger says it holds an effective claim and no
neighbour contests it; when two robots claim the same window, both apply the same
ordering and the loser withdraws.

That makes the zero-conflict result mean something. It is not enforced by a central
allocator that could not have let a conflict through; it is the outcome of
independent bots agreeing through announcements alone.

Deadlocks and lost contests are counted rather than treated as failures: standing
off in a corridor is what the priority layer exists to resolve, and the numbers give
that work something real to aim at.

*A robot that has started moving is committed.* It may re-evaluate at any tick, but
it can only adopt a new path while it is still sitting at a node. Replanning a robot
half way down an edge would be modelling something the firmware cannot do.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from pathlib import Path as FilePath

from spore_planner.p2p import Ledger, in_claim_range
from spore_planner.planner import (
    DEFAULT_CONFIG,
    Config,
    EnergyState,
    Goal,
    Path,
    PeerView,
    Planner,
    PlanStatus,
    Request,
    SelfState,
)
from spore_planner.planner.kinematics import DEFAULT_KINEMATICS
from spore_planner.warehouse import Graph, NodeType, Topology, load_map_file
from spore_planner.warehouse.map import Heading

TICK_MS = 200
"""Heartbeat period, and the simulation's time step."""

STUCK_TICKS = 150
"""Ticks without progress before a robot counts as deadlocked (30 s at 200 ms)."""


@dataclass(slots=True)
class Robot:
    bot_id: int
    node_id: int
    heading: Heading | None = None
    goal: Goal | None = None
    path: Path | None = None
    index: int = 0
    stable_for: int = 0
    moving: bool = False
    energy: EnergyState = EnergyState.OK

    completed: int = 0
    replans: int = 0
    stuck_ticks: int = 0
    deadlocks: int = 0
    failures: int = 0
    refusals: int = 0

    def arrivals(self) -> tuple[int, ...]:
        """When the robot is fully inside each node of its path."""
        if self.path is None or not self.path.hops:
            return ()
        hops = self.path.hops
        return (hops[0].t_in, *(hop.t_out for hop in hops[:-1]))


@dataclass
class SimReport:
    ticks: int = 0
    robots: int = 0
    missions_completed: int = 0
    replans: int = 0
    plan_failures: int = 0
    claims_refused: int = 0
    contests_lost: int = 0
    announcements: int = 0
    deadlocks: int = 0
    node_conflicts: list[str] = field(default_factory=list)
    swap_conflicts: list[str] = field(default_factory=list)
    starved: list[int] = field(default_factory=list)

    @property
    def clean(self) -> bool:
        """No two robots ever shared a node, and none swapped across an edge."""
        return not self.node_conflicts and not self.swap_conflicts

    def summary(self) -> str:
        return (
            f"{self.robots} robots, {self.ticks} ticks\n"
            f"  missions completed : {self.missions_completed}\n"
            f"  replans            : {self.replans}\n"
            f"  plan failures      : {self.plan_failures}\n"
            f"  claims refused     : {self.claims_refused}\n"
            f"  contests lost      : {self.contests_lost}\n"
            f"  announcements sent : {self.announcements}\n"
            f"  node conflicts     : {len(self.node_conflicts)}\n"
            f"  swap conflicts     : {len(self.swap_conflicts)}\n"
            f"  deadlocks (out of scope, counted) : {self.deadlocks}\n"
            f"  starved robots     : {len(self.starved)}"
        )


class Simulation:
    """A fleet of independently-planning robots on one map."""

    def __init__(
        self,
        graph: Graph,
        *,
        robots: int = 20,
        seed: int = 0,
        config: Config = DEFAULT_CONFIG,
        topology: Topology | None = None,
    ) -> None:
        self.graph = graph
        self.config = config
        self.topology = topology if topology is not None else Topology(graph)
        self.rng = random.Random(seed)
        self.now = 0
        self.traverse_ms = DEFAULT_KINEMATICS.cruise_ms(graph.node_spacing)
        # How long a standing robot needs to get off the node it is on: wind up from
        # rest, then cross the edge. A standing claim shorter than this is a trap --
        # a peer may legally claim the node from the instant the claim lapses, which
        # is sooner than the robot can possibly have left it, and the robot is then
        # stuck with nowhere valid to go.
        self.vacate_ms = (
            DEFAULT_KINEMATICS.half_stop_ms()
            + self.traverse_ms
            + 2 * config.safety_ms
        )
        self.report = SimReport(robots=robots)

        # Where a robot may be sent. Transfer points are the working destinations;
        # chargers and parking bays are reached through their class goals.
        self._targets = tuple(graph.nodes_of_type(NodeType.TR))

        starts = self.rng.sample(range(graph.n), robots)
        self.robots = [
            Robot(bot_id=i, node_id=graph.id_of(node)) for i, node in enumerate(starts)
        ]
        for robot in self.robots:
            robot.goal = self._new_goal(robot)

        self.planners = {
            robot.bot_id: Planner(
                graph, bot_id=robot.bot_id, config=config, topology=self.topology
            )
            for robot in self.robots
        }
        self.reach_hops = config.k_commit
        self.ledgers = {
            robot.bot_id: Ledger(
                robot.bot_id, announce_period_ms=TICK_MS, ttl_ms=3 * TICK_MS
            )
            for robot in self.robots
        }
        # node -> list of (t_in, t_out, bot_id) actually executed, for conflict checks.
        self._ledger: dict[int, list[tuple[int, int, int]]] = {}
        self._traversals: dict[tuple[int, int], list[tuple[int, int, int]]] = {}

    # -- driving -------------------------------------------------------------

    def run(self, ticks: int) -> SimReport:
        for _ in range(ticks):
            self.step()
        self.report.ticks += ticks
        self.report.starved = sorted(
            robot.bot_id for robot in self.robots if robot.completed == 0
        )
        return self.report

    def step(self) -> None:
        for robot in self.robots:
            self._advance(robot)
        for robot in self.robots:
            self._settle_contests(robot)
        for robot in self.robots:
            self._replan(robot)
            self._claim(robot)
        self._announce()
        self.now += TICK_MS

    # -- one robot -----------------------------------------------------------

    def _advance(self, robot: Robot) -> None:
        """Move the robot along its path as far as the clock allows."""
        if robot.path is None:
            robot.stuck_ticks += 1
            return
        arrivals = robot.arrivals()
        moved = False
        while robot.index + 1 < len(arrivals) and arrivals[robot.index + 1] <= self.now:
            ahead = robot.path.hops[robot.index + 1]
            ledger = self.ledgers[robot.bot_id]
            if not ledger.may_enter(ahead.node_id, ahead.t_in, ahead.t_out, self.now):
                # Either we do not hold the window ahead, or a neighbour still
                # contests it. Stay put rather than drive into it.
                break
            self._record(robot, robot.index)
            robot.index += 1
            robot.node_id = robot.path.hops[robot.index].node_id
            moved = True
        if moved:
            previous = robot.path.hops[robot.index - 1].dir
            robot.heading = previous if previous is not None else robot.heading
            robot.moving = robot.index + 1 < len(arrivals)
            robot.stuck_ticks = 0
        else:
            robot.stuck_ticks += 1

        if robot.index == len(arrivals) - 1 and robot.path is not None:
            # Arrived. Bank the last node's window and take a new mission.
            self._record(robot, robot.index)
            robot.completed += 1
            self.report.missions_completed += 1
            robot.path = None
            robot.index = 0
            robot.moving = False
            robot.stable_for = 0
            robot.goal = self._new_goal(robot)

        if robot.stuck_ticks >= STUCK_TICKS:
            # Counted, not fixed: breaking a corridor standoff is the business of the
            # layer that owns the priority ordering.
            robot.deadlocks += 1
            self.report.deadlocks += 1
            robot.stuck_ticks = 0
            robot.path = None
            robot.index = 0
            robot.goal = self._new_goal(robot)

    def _replan(self, robot: Robot) -> None:
        if not self._may_replan(robot):
            return
        request = Request(
            now=self.now,
            self_state=SelfState(
                node_id=robot.node_id,
                heading=robot.heading,
                moving=robot.moving,
                energy=robot.energy,
            ),
            goal=robot.goal,
            peers=self._peers(robot),
            current=robot.path,
            stable_for=robot.stable_for,
        )
        result = self.planners[robot.bot_id].plan(request)

        if result.status is PlanStatus.ALREADY_THERE:
            robot.completed += 1
            self.report.missions_completed += 1
            robot.goal = self._new_goal(robot)
            return
        if result.status is not PlanStatus.OK:
            robot.failures += 1
            self.report.plan_failures += 1
            # Hold, rather than keep driving a route the planner has stopped
            # endorsing. Carrying on down a stale path is how a robot ends up in a
            # node a peer has since claimed, and it would make this simulation
            # measure the wrong thing: whether old plans collide, not whether the
            # planner's current answers do.
            robot.path = None
            robot.index = 0
            robot.moving = False
            return
        if result.changed and result.path is not None:
            robot.path = result.path
            robot.index = 0
            robot.stable_for = 0
            robot.replans += 1
            self.report.replans += 1
        else:
            robot.stable_for = min(robot.stable_for + 1, self.config.stable_ticks)

    def _may_replan(self, robot: Robot) -> bool:
        """A robot already rolling down an edge cannot be given a new path."""
        if robot.path is None:
            return True
        hops = robot.path.hops
        if robot.index >= len(hops) - 1:
            return True
        departure = hops[robot.index].t_out - self.traverse_ms
        return self.now <= departure

    def _peers(self, robot: Robot) -> tuple[PeerView, ...]:
        """What this bot knows about the others, and only that.

        Reservations come from its own ledger -- announcements it actually
        received -- not from reading another robot's state. Positions come from
        the roster, which the leader distributes region-wide, so a bot still sees
        where distant peers are even when they are too far to be worth
        exchanging claims with.
        """
        claims = self.ledgers[robot.bot_id].reservations_by_bot()
        views = []
        for other in self.robots:
            if other.bot_id == robot.bot_id:
                continue
            views.append(
                PeerView(
                    bot_id=other.bot_id,
                    node_id=other.node_id,
                    speed_cm_s=DEFAULT_KINEMATICS.cruise_speed_cm_s if other.moving else 0.0,
                    reservations=claims.get(other.bot_id, ()),
                )
            )
        return tuple(views)

    def _settle_contests(self, robot: Robot) -> None:
        """Give way where the ordering says this bot loses.

        Both sides of a contest reach this conclusion independently and only one
        of them acts on it, so the node frees up without either asking the other.
        """
        ledger = self.ledgers[robot.bot_id]
        ledger.expire(self.now)
        lost = ledger.lost()
        if not lost:
            return
        robot.refusals += 1
        self.report.contests_lost += 1
        ledger.withdraw()
        # Drop the path too: it was costed against windows this bot no longer has.
        robot.path = None
        robot.index = 0
        robot.moving = False

    def _claim(self, robot: Robot) -> None:
        """Claim the committed hops ahead in this bot's own ledger."""
        ledger = self.ledgers[robot.bot_id]
        windows = [(n, a, b) for n, a, b in self._wanted_windows(robot)]
        rank = self._rank(robot)
        if not windows:
            ledger.withdraw()
            return
        if ledger.propose(windows, self.now, rank=rank):
            return

        robot.refusals += 1
        self.report.claims_refused += 1
        # A neighbour we do not outrank already holds part of the route. Hold the
        # node underfoot instead and replan next tick.
        standing = [(robot.node_id, self.now, self.now + self.vacate_ms)]
        ledger.propose(standing, self.now, rank=rank)
        robot.path = None
        robot.index = 0
        robot.moving = False

    def _rank(self, robot: Robot) -> int:
        """Right of way, exactly as the leader computes `yield_priority`:
        free 0, heading to a pickup 1, carrying cargo 2."""
        if robot.goal is None or robot.path is None:
            return 0
        return 1

    def _announce(self) -> None:
        """Every bot tells the bots in claim range what it holds.

        Who to tell comes from positions the leader's roster already carries, and
        the test is exact: a peer hears from us only when one of the nodes we hold
        is close enough that its claims could reach it. The exchange itself is bot
        to bot -- nothing here passes through a leader, which is what lets it keep
        working when there is not one.
        """
        positions = {robot.bot_id: robot.node_id for robot in self.robots}
        for robot in self.robots:
            ledger = self.ledgers[robot.bot_id]
            announcement = ledger.announcement(self.now)
            held = [c.node_id for c in ledger.mine] or [robot.node_id]
            nearby = in_claim_range(
                self.graph,
                claimed_node_ids=held,
                peers={b: n for b, n in positions.items() if b != robot.bot_id},
                reach_hops=self.reach_hops,
            )
            for other in nearby:
                self.ledgers[other].receive(announcement, self.now)
                self.report.announcements += 1
            # Anyone who has drifted out of range stops mattering.
            far = set(ledger.neighbours) - set(nearby)
            for bot_id in far:
                ledger.forget(bot_id)

    def _wanted_windows(self, robot: Robot) -> tuple[tuple[int, int, int], ...]:
        if robot.path is None:
            # Standing still is still occupying a node, and the protocol's `res[]`
            # includes the current node for exactly this reason.
            return ((robot.node_id, self.now, self.now + self.vacate_ms),)
        hops = robot.path.hops[robot.index : robot.index + robot.path.committed]
        windows = [
            (hop.node_id, hop.t_in, hop.t_out) for hop in hops if hop.t_out >= self.now
        ]
        # A robot delayed behind someone else drifts past its own planned window. It
        # is still standing on the node, so it keeps holding it -- but only until it
        # could have left. Holding longer than that would block peers from passing
        # through a node the robot is about to vacate, which costs more than it saves.
        if not windows or windows[0][0] != robot.node_id:
            windows.insert(0, (robot.node_id, self.now, self.now + self.vacate_ms))
        return tuple(windows)


    def _new_goal(self, robot: Robot) -> Goal:
        """Hand out a fresh mission.

        Transfer points are not double-booked. Two robots sent to the same dead-end
        bay is not a routing problem -- the second simply cannot get in until the
        first leaves -- and a real dispatcher would not create it. Modelling it here
        would just fill the run with unreachable goals.
        """
        roll = self.rng.random()
        if roll < 0.1:
            return Goal.charge()
        if roll < 0.2:
            return Goal.park()

        taken = {
            other.goal.node_id
            for other in self.robots
            if other.bot_id != robot.bot_id and other.goal is not None
        }
        free = [n for n in self._targets if self.graph.id_of(n) not in taken]
        pool = free or list(self._targets)
        return Goal.node(self.graph.id_of(self.rng.choice(pool)))

    # -- conflict ledger -----------------------------------------------------

    def _record(self, robot: Robot, index: int) -> None:
        """Bank a completed node occupancy and check it against every other."""
        assert robot.path is not None
        hop = robot.path.hops[index]
        node = hop.node_id
        window = (hop.t_in, hop.t_out, robot.bot_id)

        for start, end, other in self._ledger.get(node, ()):
            if other == robot.bot_id:
                continue
            if start < hop.t_out and end > hop.t_in:
                self.report.node_conflicts.append(
                    f"node {node}: bot {robot.bot_id} [{hop.t_in},{hop.t_out}] "
                    f"overlaps bot {other} [{start},{end}]"
                )
        self._ledger.setdefault(node, []).append(window)

        if index + 1 < len(robot.path.hops):
            nxt = robot.path.hops[index + 1].node_id
            depart = hop.t_out - self.traverse_ms
            for start, end, other in self._traversals.get((nxt, node), ()):
                if other != robot.bot_id and start < hop.t_out and end > depart:
                    self.report.swap_conflicts.append(
                        f"edge {node}<->{nxt}: bot {robot.bot_id} and bot {other} "
                        "crossed in opposite directions"
                    )
            self._traversals.setdefault((node, nxt), []).append(
                (depart, hop.t_out, robot.bot_id)
            )


DEFAULT_MAP = (
    FilePath(__file__).resolve().parents[3]
    / ".."
    / "spore-warehouse-layout"
    / "output"
    / "warehouse.json"
)


def run() -> None:
    """Console entry point: run a fleet and print what happened."""
    import argparse

    parser = argparse.ArgumentParser(description="Run the Spore planner simulation.")
    parser.add_argument("--map", type=FilePath, default=DEFAULT_MAP)
    parser.add_argument("--robots", type=int, default=20)
    parser.add_argument("--ticks", type=int, default=1500)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    graph = Graph(load_map_file(args.map))
    simulation = Simulation(graph, robots=args.robots, seed=args.seed)
    report = simulation.run(args.ticks)
    print(report.summary())
    for line in report.node_conflicts[:5] + report.swap_conflicts[:5]:
        print("  !", line)
