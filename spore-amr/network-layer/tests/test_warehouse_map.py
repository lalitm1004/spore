"""Regression tests pinning the shape of the real map, and loader validation.

The counts asserted here are not arbitrary: the planner's design depends on this
floor plan being a sparse corridor network with dead-end bays. If a change to
`spore-warehouse-layout` alters that shape, the planner's assumptions need
revisiting, and this test is where that should surface.
"""

from __future__ import annotations

import json
from itertools import pairwise

import pytest
from tests.planning_maps import make_map_doc

from planning import NodeType
from warehouse.map import MapError, WarehouseMap


def test_real_map_counts(real_map, real_graph):
    assert real_graph.n == 881
    assert real_map.edge_count == 952
    assert real_map.node_spacing == 200
    assert real_map.units == "cm"
    assert real_map.dimensions == (12000, 7000)
    # Consolidated from 14 to 7 upstream; the graph itself was untouched, which is
    # why every other assertion in this file still holds.
    assert len(real_map.region_ids()) == 7


def test_real_map_node_types(real_graph):
    counts = {t.value: len(real_graph.nodes_of_type(t)) for t in NodeType}
    assert counts == {"PT": 721, "TR": 61, "CH": 34, "PK": 50, "YI": 15}


def test_real_map_is_a_sparse_corridor_network(real_topology):
    # Average degree 2.16: this is a corridor network, not an open grid.
    assert real_topology.degree_histogram() == {1: 107, 2: 609, 3: 81, 4: 84}
    assert len(real_topology.junctions) == 165
    assert len(real_topology.bays) == 107


def test_every_charge_park_and_yield_node_is_a_dead_end_spur(real_graph, real_topology):
    # Drives the 180 degree reversal in the kinematics model: reaching any of these
    # commits the robot to turning around to leave.
    for node_type in (NodeType.CH, NodeType.PK, NodeType.YI):
        members = real_graph.nodes_of_type(node_type)
        assert members, f"expected some {node_type} nodes"
        assert all(i in real_topology.bays for i in members), node_type


def test_real_map_corridor_lengths(real_topology):
    hops = sorted(c.hops for c in real_topology.corridors)
    assert len(hops) == 343
    assert max(hops) == 17
    assert sum(1 for h in hops if h > 5) == 39
    assert sum(1 for h in hops if h > 10) == 8
    assert not any(c.is_cycle for c in real_topology.corridors)


def test_real_map_is_fully_connected(real_graph):
    dist = real_graph.hops_from(0)
    assert all(d != 0xFFFF for d in dist)


def test_corridor_decomposition_covers_every_edge(real_graph, real_topology):
    steps = set()
    for corridor in real_topology.corridors:
        for u, v in pairwise(corridor.nodes):
            steps.add((u, v) if u < v else (v, u))
    expected = {
        (u, v) if u < v else (v, u)
        for u in range(real_graph.n)
        for v, _ in real_graph.neighbours(u)
    }
    assert steps == expected


# -- loader validation -------------------------------------------------------


def _doc() -> dict:
    return make_map_doc({(0, 0): "PT", (1, 0): "PT", (2, 0): "TR"})


def test_loader_accepts_a_well_formed_document():
    assert WarehouseMap(_doc()).node_spacing == 200


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda d: d.__setitem__("units", "mm"), "units must be 'cm'"),
        (lambda d: d.__setitem__("node_spacing", 0), "node_spacing must be positive"),
        (lambda d: d.pop("edges"), "missing required field 'edges'"),
        (lambda d: d.__setitem__("nodes", []), "nodes must be a non-empty array"),
        (lambda d: d.__setitem__("regions", []), "regions must be a non-empty array"),
        (lambda d: d["edges"][0].__setitem__("b", 99), "unknown node id 99"),
        (lambda d: d["edges"][0].__setitem__("b", d["edges"][0]["a"]), "self-loop"),
        (lambda d: d["edges"].append(dict(d["edges"][0])), "duplicate edge"),
        (lambda d: d["edges"][0].__setitem__("length", 150), "must equal node_spacing"),
        (lambda d: d["nodes"][1]["position"].__setitem__("y", 200.0), "not an axis-aligned step"),
        (lambda d: d["nodes"][0]["position"].__setitem__("x", -5.0), "outside dimensions"),
        (lambda d: d["nodes"][0].__setitem__("region_id", 7), "unknown region_id 7"),
        (lambda d: d["nodes"][0].__setitem__("node_type", "XX"), "invalid node_type"),
        (lambda d: d["nodes"][0].__setitem__("id", d["nodes"][1]["id"]), "duplicate node id"),
        (lambda d: d["nodes"][0].__setitem__("name", ""), "name must be a non-empty string"),
        (lambda d: d["regions"][0].__setitem__("density", "packed"), "density is invalid"),
    ],
)
def test_loader_rejects_malformed_documents(mutate, message):
    doc = _doc()
    mutate(doc)
    with pytest.raises(MapError, match=message):
        WarehouseMap(doc)


def test_loader_rejects_non_json(tmp_path):
    broken = tmp_path / "warehouse.json"
    broken.write_text("{nope")
    with pytest.raises(MapError, match="not valid JSON"):
        WarehouseMap.load(broken)


def test_loader_rejects_a_non_object_document():
    with pytest.raises(MapError, match="map must be an object"):
        WarehouseMap(json.loads("[]"))


def test_graph_rejects_a_map_too_large_for_the_distance_cache(monkeypatch):
    from warehouse import map as map_module

    monkeypatch.setattr(map_module, "UNREACHABLE", 2)
    with pytest.raises(ValueError, match="addresses at most"):
        WarehouseMap(_doc())
