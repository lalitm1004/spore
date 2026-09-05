"""The lane graph a junction depends on.

`bearing` is the load-bearing part: it is what turns the network layer's "go to
node 9" into an absolute heading for the firmware to rotate to, and it is the
only place that translation happens. Nothing on the wire says left or right --
both ends hold the map -- so a wrong bearing here sends a robot down a lane
that is not there and nothing else catches it.
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


# ------------------------------------------------------------ shortest path --

def test_a_path_to_an_adjacent_node_is_one_hop():
    graph = lattice(rows=4, columns=4, spacing=2.0)

    assert graph.path(5, 6) == [5, 6]


def test_a_path_to_yourself_is_no_hops():
    graph = lattice(rows=4, columns=4, spacing=2.0)

    assert graph.path(5, 5) == [5]


def test_a_path_across_the_lattice_is_the_shortest_one():
    """Manhattan distance on a 2 m grid: node 0 to node 15 on a 4x4 is three
    rows and three columns, so seven nodes including both ends."""
    graph = lattice(rows=4, columns=4, spacing=2.0)

    route = graph.path(0, 15)

    assert route[0] == 0 and route[-1] == 15
    assert len(route) == 7


def test_every_step_of_a_path_is_a_real_lane():
    """The whole point: a robot drives this hop by hop, and a step that is not
    a lane is a robot driving through a rack."""
    graph = lattice(rows=5, columns=5, spacing=2.0)

    route = graph.path(0, 24)

    for here, nxt in zip(route, route[1:]):
        assert nxt in graph.neighbours(here)


def test_there_is_no_path_to_a_disconnected_node():
    """An island is not reachable, and saying so beats routing into it."""
    nodes = [Node(node_id=0, x=0.0, y=0.0), Node(node_id=1, x=2.0, y=0.0),
             Node(node_id=2, x=9.0, y=9.0)]
    graph = Graph(nodes, [Edge(0, 1)])

    assert graph.path(0, 2) is None


def test_next_hop_is_the_second_node_of_the_path():
    graph = lattice(rows=4, columns=4, spacing=2.0)

    assert graph.next_hop(0, 15) == graph.path(0, 15)[1]


def test_next_hop_to_where_you_already_are_is_nothing():
    """Arrival is not a move. The caller asks for a new goal instead."""
    graph = lattice(rows=4, columns=4, spacing=2.0)

    assert graph.next_hop(5, 5) is None


def test_far_nodes_are_measured_in_hops_not_straight_lines():
    """A node can be metres away across a rack and a long drive around it.
    Distance for routing has to be along the lanes."""
    graph = lattice(rows=5, columns=5, spacing=2.0)

    far = graph.far_nodes(0, minimum_hops=6)

    assert far
    for node in far:
        assert len(graph.path(0, node)) - 1 >= 6


def test_far_nodes_excludes_where_you_already_are():
    graph = lattice(rows=5, columns=5, spacing=2.0)

    assert 0 not in graph.far_nodes(0, minimum_hops=1)


def test_asking_for_further_than_the_map_allows_gives_nothing():
    """Rather than quietly returning somewhere near. The caller decides what a
    fleet does when nowhere is far enough."""
    graph = lattice(rows=3, columns=3, spacing=2.0)

    assert graph.far_nodes(0, minimum_hops=99) == []


def test_the_furthest_corner_of_a_lattice_is_the_furthest_node():
    graph = lattice(rows=4, columns=4, spacing=2.0)

    far = graph.far_nodes(0, minimum_hops=6)

    assert far == [15]      # the only node six hops from the opposite corner
