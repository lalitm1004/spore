"""Graph and topology behaviour on small, hand-checkable maps."""

from __future__ import annotations

import pytest
from conftest import line, make_graph, make_map

from spore_planner.warehouse import Graph, Heading, NodeType, Topology, quarter_turns
from spore_planner.warehouse.graph import UNREACHABLE
from spore_planner.warehouse.map import Position, heading_between


def test_quarter_turns_is_symmetric_and_capped_at_a_reversal():
    assert quarter_turns(Heading.E, Heading.E) == 0
    assert quarter_turns(Heading.E, Heading.N) == 1
    assert quarter_turns(Heading.N, Heading.E) == 1
    assert quarter_turns(Heading.E, Heading.W) == 2
    assert quarter_turns(Heading.N, Heading.S) == 2
    for a in Heading:
        for b in Heading:
            assert quarter_turns(a, b) == quarter_turns(b, a)
            assert 0 <= quarter_turns(a, b) <= 2


def test_heading_deltas_and_opposites_agree():
    for heading in Heading:
        dx, dy = heading.delta
        ox, oy = heading.opposite.delta
        assert (dx + ox, dy + oy) == (0, 0)
        assert quarter_turns(heading, heading.opposite) == 2


def test_heading_between_uses_map_coordinates_with_north_as_positive_y():
    origin = Position(0.0, 0.0)
    assert heading_between(origin, Position(200.0, 0.0)) is Heading.E
    assert heading_between(origin, Position(-200.0, 0.0)) is Heading.W
    assert heading_between(origin, Position(0.0, 200.0)) is Heading.N
    assert heading_between(origin, Position(0.0, -200.0)) is Heading.S


def test_heading_between_rejects_diagonal_and_coincident_positions():
    with pytest.raises(ValueError, match="not axis-aligned"):
        heading_between(Position(0.0, 0.0), Position(200.0, 200.0))
    with pytest.raises(ValueError, match="coincide"):
        heading_between(Position(0.0, 0.0), Position(0.0, 0.0))


def test_index_and_id_round_trip():
    graph = make_graph(line(4))
    for i in range(graph.n):
        assert graph.index(graph.id_of(i)) == i
    assert graph.has_id(0)
    assert not graph.has_id(999)
    with pytest.raises(LookupError, match="not in this map"):
        graph.index(999)


def test_dense_indices_follow_ascending_external_ids():
    graph = make_graph(line(5))
    assert list(graph.ids) == sorted(graph.ids)


def test_neighbours_are_sorted_for_deterministic_expansion():
    # A plus: centre has all four neighbours, so ordering is observable.
    cells = {(1, 1): "PT", (0, 1): "PT", (2, 1): "PT", (1, 0): "PT", (1, 2): "PT"}
    graph = make_graph(cells)
    centre = next(i for i in range(graph.n) if graph.degree(i) == 4)
    headings = [h for _, h in graph.neighbours(centre)]
    assert headings == sorted(headings)
    assert set(headings) == set(Heading)


def test_heading_to_matches_geometry():
    graph = make_graph({(0, 0): "PT", (1, 0): "PT", (0, 1): "PT"})
    origin = next(i for i in range(graph.n) if graph.degree(i) == 2)
    for neighbour, heading in graph.neighbours(origin):
        assert graph.heading_to(origin, neighbour) is heading
        assert graph.are_adjacent(origin, neighbour)


def test_heading_to_rejects_non_adjacent_nodes():
    graph = make_graph(line(3))
    assert not graph.are_adjacent(0, 2)
    with pytest.raises(LookupError, match="not adjacent"):
        graph.heading_to(0, 2)


def test_nodes_of_type_indexes_every_type():
    graph = make_graph({(0, 0): "PT", (1, 0): "CH", (2, 0): "TR"})
    assert len(graph.nodes_of_type(NodeType.CH)) == 1
    assert len(graph.nodes_of_type(NodeType.PK)) == 0
    assert len(graph.nodes_of_type(NodeType.PT)) == 1


def test_hops_from_measures_exact_graph_distance():
    graph = make_graph(line(5))
    dist = graph.hops_from(0)
    assert list(dist) == [0, 1, 2, 3, 4]


def test_hops_from_is_multi_source():
    graph = make_graph(line(5))
    dist = graph.hops_from([0, 4])
    assert list(dist) == [0, 1, 2, 1, 0]


def test_hops_from_marks_unreachable_nodes():
    # Two disjoint segments: no edge is generated between (1,0) and (3,0).
    graph = make_graph({(0, 0): "PT", (1, 0): "PT", (3, 0): "PT", (4, 0): "PT"})
    dist = graph.hops_from(0)
    assert dist[graph.index(2)] == UNREACHABLE
    assert dist[graph.index(3)] == UNREACHABLE


def test_hops_from_caches_by_source_set():
    graph = make_graph(line(4))
    first = graph.hops_from(0)
    assert graph.hops_from(0) is first
    assert graph.hops_from([0]) is first
    assert graph.hops_from([0, 1]) is not first
    graph.clear_distance_cache()
    assert graph.hops_from(0) is not first


def test_hops_from_validates_its_sources():
    graph = make_graph(line(3))
    with pytest.raises(ValueError, match="at least one source"):
        graph.hops_from([])
    with pytest.raises(IndexError, match="out of range"):
        graph.hops_from(99)


# -- topology ----------------------------------------------------------------


def test_a_straight_line_is_one_corridor_between_its_two_ends():
    topo = Topology(make_graph(line(6)))
    assert len(topo.corridors) == 1
    corridor = topo.corridors[0]
    assert corridor.hops == 5
    assert len(corridor.interior) == 4
    assert not corridor.is_cycle
    assert topo.bays == {corridor.nodes[0], corridor.nodes[-1]}
    assert topo.junctions == frozenset()


def test_a_t_junction_splits_into_three_corridors():
    cells = {(0, 0): "PT", (1, 0): "PT", (2, 0): "PT", (1, 1): "PT"}
    graph = make_graph(cells)
    topo = Topology(graph)
    assert len(topo.junctions) == 1
    assert len(topo.corridors) == 3
    assert sorted(c.hops for c in topo.corridors) == [1, 1, 1]


def test_adjacent_endpoints_form_a_corridor_with_no_interior():
    cells = {(0, 0): "PT", (1, 0): "PT", (2, 0): "PT", (1, 1): "PT", (1, 2): "PT"}
    topo = Topology(make_graph(cells))
    short = [c for c in topo.corridors if c.hops == 1]
    assert short and all(c.interior == () for c in short)


def test_a_pure_ring_is_still_decomposed():
    # No node has degree != 2, so the endpoint walk finds nothing and the cycle
    # fallback has to catch it.
    cells = {(x, y): "PT" for x in range(3) for y in range(3) if (x, y) != (1, 1)}
    topo = Topology(make_graph(cells))
    assert topo.degree_histogram() == {2: 8}
    assert len(topo.corridors) == 1
    assert topo.corridors[0].is_cycle
    assert topo.corridors[0].hops == 8


def test_corridor_entered_by_names_the_committed_corridor():
    cells = {(0, 0): "PT", (1, 0): "PT", (2, 0): "PT", (1, 1): "PT"}
    graph = make_graph(cells)
    topo = Topology(graph)
    junction = next(iter(topo.junctions))
    neighbour = graph.neighbours(junction)[0][0]
    corridor = topo.corridor_entered_by(junction, neighbour)
    assert corridor is not None
    assert junction in corridor.endpoints()
    assert topo.corridor_entered_by(junction, junction) is None


def test_longest_corridor_picks_the_deepest_run():
    cells = {(x, 0): "PT" for x in range(6)}
    cells[(2, 1)] = "PT"
    topo = Topology(make_graph(cells))
    assert topo.longest_corridor().hops == 3
    assert Topology(Graph(make_map({(0, 0): "PT", (1, 0): "PT"}))).longest_corridor().hops == 1


def test_bay_and_junction_predicates_agree_with_degree():
    graph = make_graph({(0, 0): "PT", (1, 0): "PT", (2, 0): "PT", (1, 1): "CH"})
    topo = Topology(graph)
    for i in range(graph.n):
        assert topo.is_bay(i) == (graph.degree(i) == 1)
        assert topo.is_junction(i) == (graph.degree(i) >= 3)
