"""
Task 6 Validation — A* pathfinding (PathPlanning)
Run from the project root:  pytest tests/test_task6_astar.py -v

Tests that your A* implementation finds valid shortest paths through the graph.
Requires Task 4 (Graph) to be implemented first.
All tests must pass before moving on to Task 7.
"""
import sys
import os
import json
import math
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from fleet_management.graph import Graph
from fleet_management.fleet_management import PathPlanning


@pytest.fixture(scope="module")
def graph():
    with open("data/input_files/lif_file.json") as f:
        lif_data = json.load(f)
    return Graph(lif_data=lif_data)


@pytest.fixture(scope="module")
def path_planning(graph):
    with open("data/input_files/config_file.json") as f:
        config_data = json.load(f)
    return PathPlanning(config_data=config_data, graph=graph)


# ── Helper: validate a returned path ────────────────────────────────────────

def assert_valid_path(path_nodes, path_edges, start, goal, graph):
    assert path_nodes is not None, f"A* returned None for path {start}→{goal}"
    assert path_edges is not None, f"A* returned None for edges {start}→{goal}"
    assert len(path_nodes) >= 1, "Path must have at least one node"
    assert path_nodes[0] == start, f"Path must start at {start}, got {path_nodes[0]}"
    assert path_nodes[-1] == goal, f"Path must end at {goal}, got {path_nodes[-1]}"
    assert len(path_edges) == len(path_nodes) - 1, \
        f"Expected {len(path_nodes)-1} edges for {len(path_nodes)} nodes, got {len(path_edges)}"
    for i in range(len(path_nodes) - 1):
        edge = graph.get_connected_edge(path_nodes[i], path_nodes[i + 1])
        assert edge is not None, \
            f"No edge between {path_nodes[i]} and {path_nodes[i+1]} in the graph"
        assert edge == path_edges[i], \
            f"Step {i}: expected edge {edge} between {path_nodes[i]}→{path_nodes[i+1]}, got {path_edges[i]}"


# ── Heuristic and distance ───────────────────────────────────────────────────

def test_get_h_same_node(path_planning):
    assert path_planning.get_h("N1", "N1") == 0.0, "h(x, x) must be 0"

def test_get_h_positive(path_planning):
    h = path_planning.get_h("N5", "N4")
    assert h > 0.0, "h must be positive for different nodes"

def test_get_h_is_euclidean(path_planning, graph):
    assert graph.nodes is not None, "graph.nodes is None — implement Task 4 (Graph) first"
    n5 = graph.nodes["N5"]["pos"]
    n4 = graph.nodes["N4"]["pos"]
    expected = math.dist(n5, n4)
    h = path_planning.get_h("N5", "N4")
    assert abs(h - expected) < 0.001, \
        f"get_h should return Euclidean distance {expected:.4f}, got {h:.4f}"

def test_get_distance_direct(path_planning, graph):
    assert graph.nodes is not None, "graph.nodes is None — implement Task 4 (Graph) first"
    n5 = graph.nodes["N5"]["pos"]
    n1 = graph.nodes["N1"]["pos"]
    expected = math.dist(n5, n1)
    dist = path_planning.get_distance("N5", "N1")
    assert abs(dist - expected) < 0.001, \
        f"get_distance(N5, N1) should be {expected:.4f}, got {dist:.4f}"


# ── Path validity ────────────────────────────────────────────────────────────

def test_astar_direct_edge(path_planning, graph):
    nodes, edges = path_planning.astar_search("N5", "N1")
    assert_valid_path(nodes, edges, "N5", "N1", graph)

def test_astar_N5_to_N1_is_shortest(path_planning):
    nodes, edges = path_planning.astar_search("N5", "N1")
    assert nodes is not None, "A* returned None for path N5→N1"
    assert len(nodes) == 2, \
        f"N5→N1 is a direct edge — shortest path has 2 nodes, got {len(nodes)}: {nodes}"

def test_astar_N5_to_N2(path_planning, graph):
    nodes, edges = path_planning.astar_search("N5", "N2")
    assert_valid_path(nodes, edges, "N5", "N2", graph)

def test_astar_N5_to_N3(path_planning, graph):
    nodes, edges = path_planning.astar_search("N5", "N3")
    assert_valid_path(nodes, edges, "N5", "N3", graph)

def test_astar_N5_to_N4(path_planning, graph):
    nodes, edges = path_planning.astar_search("N5", "N4")
    assert_valid_path(nodes, edges, "N5", "N4", graph)

def test_astar_N4_to_N2(path_planning, graph):
    nodes, edges = path_planning.astar_search("N4", "N2")
    assert_valid_path(nodes, edges, "N4", "N2", graph)

def test_astar_N3_to_N2(path_planning, graph):
    nodes, edges = path_planning.astar_search("N3", "N2")
    assert_valid_path(nodes, edges, "N3", "N2", graph)

def test_astar_same_node(path_planning):
    nodes, edges = path_planning.astar_search("N5", "N5")
    assert nodes == ["N5"], f"Path from N5 to N5 should be ['N5'], got {nodes}"
    assert edges == [], f"No edges needed for start == goal, got {edges}"

def test_astar_all_station_pairs(path_planning, graph):
    """A* must find a valid path between every pair of station nodes."""
    station_nodes = ["N1", "N2", "N3", "N4"]
    for start in station_nodes:
        for goal in station_nodes:
            if start == goal:
                continue
            nodes, edges = path_planning.astar_search(start, goal)
            assert_valid_path(nodes, edges, start, goal, graph)

def test_astar_from_dwelling_to_stations(path_planning, graph):
    """The dwelling node N5 must have a valid path to every station node."""
    for station in ["N1", "N2", "N3", "N4"]:
        nodes, edges = path_planning.astar_search("N5", station)
        assert_valid_path(nodes, edges, "N5", station, graph)


# ── Optimality ───────────────────────────────────────────────────────────────

def test_astar_does_not_revisit_nodes(path_planning):
    nodes, _ = path_planning.astar_search("N5", "N2")
    assert nodes is not None, "A* returned None for path N5→N2"
    assert len(nodes) == len(set(nodes)), \
        f"Path N5→N2 contains repeated nodes: {nodes}. A* should find a loop-free shortest path."
