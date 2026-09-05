"""The lane graph, and the turn resolution a junction depends on.

`turns_from` is the load-bearing part: it is what converts the network layer's
"go left" into a node to drive to, and a wrong answer sends a robot down a lane
that is not there.
"""

import math

import pytest

from tools.track.graph import Edge, Graph, Node, lattice, wrap_pi


def deg(radians):
    return round(math.degrees(radians))


# --------------------------------------------------------------- structure --

def test_lattice_has_the_expected_shape():
    graph = lattice(rows=4, columns=4, spacing=2.0)

    assert len(graph.nodes) == 16
    assert len(graph.edges) == 24          # 4*3 horizontal + 4*3 vertical
    assert graph.total_length == pytest.approx(48.0)


def test_lattice_is_centred_on_the_origin():
    graph = lattice(rows=4, columns=4, spacing=2.0)
    xs = [n.x for n in graph.nodes.values()]
    ys = [n.y for n in graph.nodes.values()]

    assert min(xs) == pytest.approx(-3.0) and max(xs) == pytest.approx(3.0)
    assert min(ys) == pytest.approx(-3.0) and max(ys) == pytest.approx(3.0)


def test_node_ids_are_row_major():
    """`id // columns` is the row, so a log line is readable without a map."""
    graph = lattice(rows=4, columns=4, spacing=2.0)

    assert graph.nodes[0].x == pytest.approx(-3.0)
    assert graph.nodes[3].x == pytest.approx(3.0)     # end of the first row
    assert graph.nodes[4].y > graph.nodes[0].y        # start of the second


def test_degrees_are_what_a_lattice_implies():
    graph = lattice(rows=4, columns=4, spacing=2.0)
    degrees = sorted(graph.degree(n) for n in graph.nodes)

    assert degrees.count(2) == 4    # corners
    assert degrees.count(3) == 8    # edges
    assert degrees.count(4) == 4    # interior junctions


def test_regions_partition_the_floor():
    graph = lattice(rows=4, columns=4, spacing=2.0)
    regions = {n.region_id for n in graph.nodes.values()}

    assert regions == {1, 2, 3, 4}
    assert all(n.region_id > 0 for n in graph.nodes.values())


def test_malformed_graphs_are_rejected():
    a, b = Node(0, 0, 0), Node(1, 1, 0)
    with pytest.raises(ValueError, match="duplicate node"):
        Graph([a, Node(0, 5, 5)], [])
    with pytest.raises(ValueError, match="unknown node"):
        Graph([a, b], [Edge(0, 9)])
    with pytest.raises(ValueError, match="self-loop"):
        Graph([a, b], [Edge(0, 0)])
    with pytest.raises(ValueError, match="duplicate edge"):
        Graph([a, b], [Edge(0, 1), Edge(1, 0)])
    with pytest.raises(ValueError, match="at least 2x2"):
        lattice(rows=1, columns=4, spacing=2.0)


# ------------------------------------------------------------ lane bearing --

def test_bearing_is_the_lane_direction():
    graph = lattice(rows=4, columns=4, spacing=2.0)

    assert deg(graph.bearing(0, 1)) == 0       # east
    assert deg(graph.bearing(1, 0)) == 180     # west
    assert deg(graph.bearing(0, 4)) == 90      # north
    assert deg(graph.bearing(4, 0)) == -90     # south


def test_edges_are_straight_so_bearing_is_exact():
    """This is why the shared QR schema needs no bearing field: on a straight
    span, node positions give the lane's direction exactly."""
    graph = lattice(rows=4, columns=4, spacing=2.0)
    assert graph.length(0, 1) == pytest.approx(2.0)
    assert graph.length(0, 4) == pytest.approx(2.0)


# ---------------------------------------------------------------- turning --

def test_interior_junction_offers_all_three_turns():
    graph = lattice(rows=4, columns=4, spacing=2.0)
    # Node 5 is interior. Arrive heading east, having come from node 4.
    turns = graph.turns_from(5, heading=0.0)

    assert turns == {"left": 9, "straight": 6, "right": 1}


def test_the_lane_we_arrived_on_is_never_offered():
    """A robot that has just driven in has no business turning back, and on a
    one-way lane it would be a wrong-way entry."""
    graph = lattice(rows=4, columns=4, spacing=2.0)

    for heading, arrived_from in ((0.0, 4), (math.pi / 2, 1), (math.pi, 6)):
        turns = graph.turns_from(5, heading=heading)
        assert arrived_from not in turns.values()


def test_a_corner_offers_only_what_exists():
    graph = lattice(rows=4, columns=4, spacing=2.0)
    # Node 0 is the bottom-left corner: lanes east (to 1) and north (to 4).
    # Arriving heading east, the way back is west, and there is no lane there
    # -- so both remaining lanes are on offer and only "right" is a wall.
    assert graph.turns_from(0, heading=0.0) == {"straight": 1, "left": 4}

    # Arriving heading north instead, the lane to 4 is straight ahead and the
    # lane to 1 is behind-right, outside tolerance.
    assert graph.turns_from(0, heading=math.pi / 2) == {"straight": 4, "right": 1}


def test_an_edge_node_offers_two():
    graph = lattice(rows=4, columns=4, spacing=2.0)
    # Node 1 is on the bottom edge: lanes to 0, 2 and 5.
    turns = graph.turns_from(1, heading=0.0)   # arrived heading east from 0

    assert turns == {"straight": 2, "left": 5}


def test_every_turn_leads_to_a_real_neighbour():
    """The property that matters: whatever the network layer picks, the robot
    can actually drive it."""
    graph = lattice(rows=4, columns=4, spacing=2.0)

    for node_id in graph.nodes:
        for heading_deg in (0, 90, 180, 270):
            turns = graph.turns_from(node_id, heading=math.radians(heading_deg))
            for turn, neighbour in turns.items():
                assert neighbour in graph.neighbours(node_id), \
                    "node {} heading {} offered {} -> {}, not a neighbour".format(
                        node_id, heading_deg, turn, neighbour)


def test_no_neighbour_is_offered_twice():
    """A lane can be the closest match for two ideal bearings; only the better
    fit may keep it, or the robot is told two turns go the same way."""
    graph = lattice(rows=4, columns=4, spacing=2.0)

    for node_id in graph.nodes:
        for heading_deg in (0, 45, 90, 135, 180, 225, 270, 315):
            turns = graph.turns_from(node_id, heading=math.radians(heading_deg))
            targets = list(turns.values())
            assert len(targets) == len(set(targets)), \
                "node {} heading {} offered {}".format(node_id, heading_deg, turns)


def test_a_diagonal_approach_matches_nothing_beyond_tolerance():
    """Arriving at 45 degrees, no lane is within 45 degrees of straight ahead
    on both sides -- the resolver must not invent one."""
    graph = lattice(rows=4, columns=4, spacing=2.0)
    turns = graph.turns_from(5, heading=math.radians(45))

    for neighbour in turns.values():
        assert neighbour in graph.neighbours(5)


# ----------------------------------------------------------- ground truth --

def test_distance_to_lane_is_zero_on_a_lane():
    graph = lattice(rows=4, columns=4, spacing=2.0)

    assert graph.distance_to_lane(-3.0, -3.0) == pytest.approx(0.0)   # a node
    assert graph.distance_to_lane(-2.0, -3.0) == pytest.approx(0.0)   # mid-span


def test_distance_to_lane_is_unsigned():
    """A graph has no inside, so there is no side for a sign to name."""
    graph = lattice(rows=4, columns=4, spacing=2.0)
    # Either side of the bottom-left span, same distance.
    assert graph.distance_to_lane(-2.0, -3.2) == pytest.approx(0.2)
    assert graph.distance_to_lane(-2.0, -2.8) == pytest.approx(0.2)


def test_nearest_node_finds_the_right_one():
    graph = lattice(rows=4, columns=4, spacing=2.0)
    assert graph.nearest_node(-2.9, -2.9).node_id == 0
    assert graph.nearest_node(0.1, 0.1).node_id == 10


def test_wrap_keeps_angles_in_one_turn():
    # +pi and -pi are the same angle, so only the magnitude is meaningful at
    # the boundary.
    assert abs(wrap_pi(3 * math.pi)) == pytest.approx(math.pi)
    assert abs(wrap_pi(-3 * math.pi)) == pytest.approx(math.pi)
    assert wrap_pi(0.5) == pytest.approx(0.5)
    assert wrap_pi(2 * math.pi + 0.5) == pytest.approx(0.5)


def test_a_dead_end_offers_the_way_back():
    """Every charging bay in the real warehouse is a degree-1 spur. Excluding
    the lane the robot arrived on leaves nothing, and a robot offered no turn
    sits in the bay for the rest of the run."""
    graph = Graph([Node(0, 0.0, 0.0, kind="CH"), Node(1, 0.0, 2.0)], [Edge(0, 1)])

    # Arrived heading north, into the bay. The only lane is back south.
    turns = graph.turns_from(0, heading=math.pi / 2)
    assert turns, "a dead end must still offer a way out"
    assert set(turns.values()) == {1}


def test_a_through_node_still_refuses_to_double_back():
    """The dead-end allowance must not become a general licence to reverse."""
    graph = lattice(rows=4, columns=4, spacing=2.0)
    assert 4 not in graph.turns_from(5, heading=0.0).values()
