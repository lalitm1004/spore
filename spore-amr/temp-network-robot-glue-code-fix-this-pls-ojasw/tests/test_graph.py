"""The map, as the network layer needs it.

Less than the robot's graph on purpose: what connects to what, and how far
apart things are in lanes. No bearings -- the network layer names nodes, never
directions.
"""

import json

from temp_network_interface.graph import Graph, load_map


def ring(size):
    nodes = [{"id": i} for i in range(size)]
    edges = [(i, (i + 1) % size) for i in range(size)]
    return Graph(nodes, edges)


def test_lanes_run_both_ways():
    graph = Graph([{"id": 0}, {"id": 1}], [(0, 1)])

    assert graph.neighbours(0) == [1]
    assert graph.neighbours(1) == [0]


def test_distance_is_counted_in_hops():
    graph = ring(10)

    assert graph.hops(0, 3) == 3
    assert graph.hops(0, 7) == 3        # the short way round


def test_an_unreachable_node_has_no_distance():
    graph = Graph([{"id": 0}, {"id": 1}, {"id": 2}], [(0, 1)])

    assert graph.hops(0, 2) is None


def test_far_nodes_are_at_least_that_far():
    graph = ring(20)

    for node in graph.far_nodes(0, minimum_hops=5):
        assert graph.hops(0, node) >= 5


def test_far_nodes_never_includes_where_you_are():
    assert 0 not in ring(20).far_nodes(0, minimum_hops=1)


def test_nowhere_far_enough_is_empty_not_nearby():
    """Quietly returning somewhere near would send a robot two nodes down the
    aisle and call it a mission."""
    assert ring(6).far_nodes(0, minimum_hops=99) == []


def test_an_edge_naming_an_unknown_node_is_ignored():
    """A map is data from another tool; a dangling edge must not crash the
    fleet's router on startup."""
    graph = Graph([{"id": 0}, {"id": 1}], [(0, 1), (1, 99)])

    assert graph.neighbours(1) == [0]


def test_a_map_round_trips_from_json(tmp_path):
    path = tmp_path / "warehouse.json"
    path.write_text(json.dumps({
        "nodes": [{"id": 1}, {"id": 2}, {"id": 3}],
        "edges": [{"a": 1, "b": 2}, {"a": 2, "b": 3}],
    }))

    graph = load_map(path)

    assert graph.neighbours(2) == [1, 3]
    assert graph.hops(1, 3) == 2
