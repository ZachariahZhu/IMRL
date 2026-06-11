"""
Task 4 Validation — Graph class
Run from the workspace root:  pytest fleet_management/tests/test_task4_graph.py -v

Tests that your Graph class correctly reads the layout from lif_file.json.
All tests must pass before moving on to Task 5.
"""
import sys
import os
import json
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data', 'input_files')
from fleet_management.graph import Graph


@pytest.fixture(scope="module")
def graph():
    with open(os.path.join(_DATA_DIR, "lif_file.json")) as f:
        lif_data = json.load(f)
    return Graph(lif_data=lif_data)


# ── Nodes ────────────────────────────────────────────────────────────────────

def test_node_count(graph):
    assert graph.nodes is not None, "graph.nodes is None — implement get_nodes() first"
    assert len(graph.nodes) == 14, f"Expected 14 nodes (N1–N14), got {len(graph.nodes)}"

def test_node_ids_present(graph):
    assert graph.nodes is not None, "graph.nodes is None — implement get_nodes() first"
    for i in range(1, 15):
        assert f"N{i}" in graph.nodes, f"Node N{i} missing from graph.nodes"

def test_node_positions(graph):
    assert graph.nodes is not None, "graph.nodes is None — implement get_nodes() first"
    expected = {
        "N1": (2.46, 0.79),
        "N2": (2.46, 3.70),
        "N3": (5.45, 2.25),
        "N4": (5.55, 4.75),
        "N5": (1.05, 0.90),
        "N7": (2.46, 2.25),
        "N8": (4.00, 2.25),
        "N9": (4.70, 0.80),
    }
    for node_id, (x, y) in expected.items():
        pos = graph.nodes[node_id]["pos"]
        assert abs(pos[0] - x) < 0.01 and abs(pos[1] - y) < 0.01, \
            f"Node {node_id}: expected pos ({x}, {y}), got {pos}"


# ── Edges ────────────────────────────────────────────────────────────────────

def test_edges_not_empty(graph):
    assert graph.edges is not None, "graph.edges is None — implement get_edges() first"
    assert len(graph.edges) > 0, "graph.edges must not be empty"

def test_edge_fields(graph):
    assert graph.edges is not None, "graph.edges is None — implement get_edges() first"
    for edge_id, edge in graph.edges.items():
        assert "startNodeId" in edge, f"Edge {edge_id} missing 'startNodeId'"
        assert "endNodeId" in edge, f"Edge {edge_id} missing 'endNodeId'"
        assert "startNodePos" in edge, f"Edge {edge_id} missing 'startNodePos'"
        assert "endNodePos" in edge, f"Edge {edge_id} missing 'endNodePos'"

def test_known_edge_E21(graph):
    assert graph.edges is not None, "graph.edges is None — implement get_edges() first"
    assert "E21" in graph.edges, "Edge E21 (N5↔N1) must be in graph.edges"
    e = graph.edges["E21"]
    endpoints = {e["startNodeId"], e["endNodeId"]}
    assert endpoints == {"N5", "N1"}, f"E21 should connect N5 and N1, got {endpoints}"


# ── Stations ─────────────────────────────────────────────────────────────────

def test_station_ids(graph):
    assert graph.stations is not None, "graph.stations is None — implement get_stations() first"
    for s in ["S1", "S2", "S3", "S4"]:
        assert s in graph.stations, f"Station {s} missing from graph.stations"

def test_station_interaction_nodes(graph):
    assert graph.stations is not None, "graph.stations is None — implement get_stations() first"
    assert "N1" in graph.stations["S1"]["interactionNodeIds"], "S1 must map to N1"
    assert "N2" in graph.stations["S2"]["interactionNodeIds"], "S2 must map to N2"
    assert "N3" in graph.stations["S3"]["interactionNodeIds"], "S3 must map to N3"
    assert "N4" in graph.stations["S4"]["interactionNodeIds"], "S4 must map to N4"

def test_station_descriptions(graph):
    assert graph.stations is not None, "graph.stations is None — implement get_stations() first"
    assert graph.stations["S1"]["stationDescription"] == "PROCESS", "S1 must be PROCESS"
    assert graph.stations["S2"]["stationDescription"] == "TRANSFER", "S2 must be TRANSFER"
    assert graph.stations["S3"]["stationDescription"] == "PROCESS", "S3 must be PROCESS"
    assert graph.stations["S4"]["stationDescription"] == "TRANSFER", "S4 must be TRANSFER"

def test_no_charging_in_stations(graph):
    assert graph.stations is not None, "graph.stations is None — implement get_stations() first"
    for station in graph.stations.values():
        assert station["stationDescription"] != "CHARGING", \
            "CHARGING stations should be in dwelling_nodes, not in stations"


# ── Dwelling nodes ───────────────────────────────────────────────────────────

def test_dwelling_node_count(graph):
    assert graph.dwelling_nodes is not None, \
        "graph.dwelling_nodes is None — implement get_dwelling_nodes() first"
    assert len(graph.dwelling_nodes) >= 4, \
        f"Expected at least 4 dwelling nodes (N5, N9, N10, N14), got {len(graph.dwelling_nodes)}"

def test_dwelling_node_ids(graph):
    assert graph.dwelling_nodes is not None, \
        "graph.dwelling_nodes is None — implement get_dwelling_nodes() first"
    for n in ["N5", "N9", "N10", "N14"]:
        assert n in graph.dwelling_nodes, f"Dwelling node {n} missing from graph.dwelling_nodes"


# ── Graph helper methods ─────────────────────────────────────────────────────

def test_get_connected_nodes_N5(graph):
    connected = graph.get_connected_nodes("N5")
    assert connected is not None, "get_connected_nodes must not return None"
    assert "N1" in connected, "N5 should be connected to N1 via E21"

def test_get_connected_nodes_N1(graph):
    connected = graph.get_connected_nodes("N1")
    assert connected is not None, "get_connected_nodes must not return None"
    assert "N5" in connected, "N1 should be connected to N5 (bidirectional)"
    assert "N7" in connected, "N1 should be connected to N7 via E1"

def test_get_connected_edge_N5_N1(graph):
    edge = graph.get_connected_edge("N5", "N1")
    assert edge == "E21", f"Edge between N5 and N1 should be E21, got {edge}"

def test_get_connected_edge_bidirectional(graph):
    edge_fwd = graph.get_connected_edge("N5", "N1")
    edge_bwd = graph.get_connected_edge("N1", "N5")
    assert edge_fwd is not None, \
        "get_connected_edge('N5','N1') returned None — implement get_connected_edge() first"
    assert edge_fwd == edge_bwd, \
        f"get_connected_edge should return the same edge in both directions: {edge_fwd} vs {edge_bwd}"
