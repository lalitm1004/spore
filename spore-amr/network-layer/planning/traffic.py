"""What the other robots are doing, in three tiers of confidence.

WHAT
    `TrafficView` — everything the search needs to know about other robots at
    one instant: which node-time windows are taken, which nodes are impassable,
    and which are merely expensive.

WHERE
    Built once per decision by `planning.decide`, from the reservation ledger
    (`reservations.ledger`) and the roster (`peers.table`). Handed to
    `planning.sipp` as constraints and cost.

WHY
    A robot learns about its neighbours two very different ways, and conflating
    them would be wrong in both directions. A **declared** claim is a promise:
    the peer has said it is taking that node, and blocking it is correct. A
    **polled** position is an observation: the peer is standing there, and where
    it goes next is our inference, not its word.

    So the tiers are ranked, and the ranking is strict:

      1. Declared -- peer `res[]` from the ledger. Hard.
      2. Predicted -- extrapolated from `node_trail`. Hard, but never applied to
         a peer that has declared anything.
      3. Soft -- positions, region density, obstructions. Cost only.

    **A prediction never contradicts a declaration.** If a peer has told us it
    holds A and B, we do not additionally block C because we guessed it was
    heading there; it has told us what it intends and that beats our inference.
    Prediction exists only to cover peers that have not announced -- out of
    claim range, freshly arrived, or moving between announcements.

HOW — why prediction is worth trusting at all
    On this floor plan heading usually *determines* the next hops rather than
    hinting at them. Measured over all 1,904 directed steps of the real map,
    64% have at least one next hop with no choice at all, 41% have two, and the
    longest forced run is 16. Inside a corridor there is nowhere else to go:
    that is a fact about the graph, not a guess about the driver.

    So a prediction runs only as far as the first junction, where the peer
    regains a real choice. Past that it becomes tier-3 cost and stops blocking
    anything.

HOW — implementation
    Predictions are expressed as ordinary `Reservation`s on a synthetic
    `PeerView`, so tier 1 and tier 2 flow through the same interval machinery
    (`planning.intervals`) and the search cannot tell them apart. Precedence is
    then simply a matter of not generating tier 2 for a peer that has tier 1.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from planning import congestion as congestion_module
from planning.geometry import Heading, heading_between
from planning.graph import Graph
from planning.intervals import ReservationTable
from planning.kinematics import Kinematics
from planning.types import Config, Obstruction, PeerView, RegionGossip, Reservation


@dataclass(frozen=True, slots=True)
class Observation:
    """One peer as this robot currently sees it.

    Built by the caller from the roster and the ledger, so `planning` never
    reaches into either.
    """

    bot_id: int
    node_id: int | None = None
    trail: tuple[int, ...] = ()
    """Recent distinct QR nodes, newest first — `node_trail` from the roster.
    Two entries give a heading; that is the whole basis of tier 2."""

    reservations: tuple[Reservation, ...] = ()
    rank: int = 0
    """`yield_priority`: free 0 < heading-to-pickup 1 < carrying cargo 2."""


def predict(
    graph: Graph,
    observation: Observation,
    *,
    now: int,
    traverse_ms: int,
    max_hops: int,
) -> tuple[Reservation, ...]:
    """Where a peer must go next, given where it came from.

    Empty when the peer has declared reservations (tier 1 wins), when the trail
    is too short to give a heading, or when it is already standing at a junction
    and could go anywhere.

    Windows are deliberately generous — one traversal either side of the
    expected arrival — because this is an inference about a robot that never
    promised us anything, and being early or late by a hop is entirely normal.
    """
    if observation.reservations:
        return ()
    if len(observation.trail) < 2:
        return ()

    newest, previous = observation.trail[0], observation.trail[1]
    if not (graph.has_id(newest) and graph.has_id(previous)):
        return ()
    here, came_from = graph.index(newest), graph.index(previous)
    if not graph.are_adjacent(came_from, here):
        # A gap in the trail: a missed scan, or the peer was carried. A bearing
        # across a gap is not a heading.
        return ()

    heading: Heading = heading_between(graph.position[came_from], graph.position[here])

    predicted: list[Reservation] = []
    previous_index, current = came_from, here
    for step in range(1, max_hops + 1):
        if graph.degree(current) != 2:
            break  # a real choice: we stop guessing here
        nxt = next(v for v, _ in graph.neighbours(current) if v != previous_index)
        arrival = now + step * traverse_ms
        predicted.append(
            Reservation(
                node_id=graph.id_of(nxt),
                t_in=arrival - traverse_ms,
                t_out=arrival + traverse_ms,
                dir=heading,
            )
        )
        previous_index, current = current, nxt

    return tuple(predicted)


@dataclass(frozen=True, slots=True)
class TrafficView:
    """The three tiers, resolved into what the search actually consumes."""

    table: ReservationTable
    field: congestion_module.CongestionField
    peers: tuple[PeerView, ...] = ()
    """The peers as the search must see them — declared claims where a peer gave
    us any, predicted ones where it did not. Handed to `Planner.plan` so the
    search reasons about exactly the traffic this view describes."""

    predicted_for: frozenset[int] = field(default_factory=frozenset)
    """Bot ids we are guessing about rather than quoting. Diagnostics only —
    useful when explaining why a robot waited for someone who never came."""

    def safe_intervals(self, node: int):
        return self.table.safe_intervals(node)

    def is_free(self, node: int, start: int, end: int) -> bool:
        return self.table.is_free(node, start, end)

    def blockers(self, node: int, start: int, end: int) -> tuple[int, ...]:
        return self.table.blockers(node, start, end)

    def interval_containing(self, node: int, t: int):
        return self.table.interval_containing(node, t)

    @property
    def blocked(self) -> frozenset[int]:
        return self.field.blocked

    def penalty(self, node: int) -> float:
        return self.field(node)


def build(
    graph: Graph,
    observations: tuple[Observation, ...],
    *,
    now: int,
    config: Config,
    kinematics: Kinematics,
    hop_cost: float,
    obstructions: tuple[Obstruction, ...] = (),
    gossip: tuple[RegionGossip, ...] = (),
    exclude_bot_id: int | None = None,
) -> TrafficView:
    """Assemble the traffic picture for one decision."""
    traverse_ms = kinematics.cruise_ms(graph.node_spacing)

    views: list[PeerView] = []
    predicted_for: set[int] = set()
    for observation in sorted(observations, key=lambda o: o.bot_id):
        if exclude_bot_id is not None and observation.bot_id == exclude_bot_id:
            continue
        claims = observation.reservations
        if not claims:
            claims = predict(
                graph,
                observation,
                now=now,
                traverse_ms=traverse_ms,
                max_hops=config.k_commit,
            )
            if claims:
                predicted_for.add(observation.bot_id)
        views.append(
            PeerView(
                bot_id=observation.bot_id,
                reservations=claims,
                node_id=observation.node_id,
            )
        )

    peers = tuple(views)
    return TrafficView(
        peers=peers,
        table=ReservationTable(
            graph, peers, now=now, config=config, exclude_bot_id=exclude_bot_id
        ),
        field=congestion_module.build(
            graph,
            config=config,
            hop_cost=hop_cost,
            peers=peers,
            gossip=gossip,
            obstructions=obstructions,
            exclude_bot_id=exclude_bot_id,
        ),
        predicted_for=frozenset(predicted_for),
    )
