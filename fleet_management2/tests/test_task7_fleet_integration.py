"""
Task 7 Validation — Fleet management integration
Run from the workspace root:  pytest fleet_management/tests/test_task7_fleet_integration.py -v

Tests that the three FleetManagement helper methods work correctly together:
  - build_path_for_task()   chains A* legs to cover all task stations
  - build_order_nodes()     assigns VDA 5050 actions to each path node
  - build_order_edges()     produces the edge dicts for generate_order_message()

Requires Tasks 4 (Graph) and 6 (A*) to be implemented first.
All tests must pass before moving on to Task 8.
"""
import sys
import os
import json
import math
import time
import pytest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data', 'input_files')

# Stub paho-mqtt so tests run without a broker connection
for _mod in ('paho', 'paho.mqtt', 'paho.mqtt.client'):
    sys.modules.setdefault(_mod, MagicMock())


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def graph():
    from fleet_management.graph import Graph
    with open(os.path.join(_DATA_DIR, "lif_file.json")) as f:
        lif_data = json.load(f)
    return Graph(lif_data=lif_data)


@pytest.fixture(scope="module")
def fm(graph):
    """
    FleetManagement instance with __init__ bypassed so no daemon thread starts.
    Path planning uses a real Graph and PathPlanning.
    MQTT calls are mocked.
    """
    with patch("vda5050_interface.mqtt_clients.mqtt_subscriber.MQTTSubscriber"), \
         patch("vda5050_interface.mqtt_clients.mqtt_publisher.MQTTPublisher"), \
         patch("vda5050_interface.interfaces.order_interface.OrderInterface"):

        from fleet_management.fleet_management import FleetManagement, PathPlanning

        with open(os.path.join(_DATA_DIR, "config_file.json")) as f:
            config_data = json.load(f)

        with patch.object(FleetManagement, '__init__', lambda *a, **kw: None):
            instance = FleetManagement.__new__(FleetManagement)

        instance.graph = graph
        instance.path_planning = PathPlanning(config_data=config_data, graph=graph)

        mock_agent = MagicMock()
        mock_agent.agentId = "mouse001"
        mock_agent.agent_state = "IDLE"
        mock_agent.current_node = "N5"
        mock_agent.current_task = None
        mock_agent.order_interface = MagicMock()

        mock_agents = MagicMock()
        mock_agents.agents = [mock_agent]
        mock_agents.order_header_id = 1

        instance.agents = mock_agents

        task_list = [
            {
                'task_id': 'T1',
                'stations': [
                    {"nodeId": "N2", "actionType": "pick"},
                    {"nodeId": "N3", "actionType": "process", "processingTime": 1.0},
                    {"nodeId": "N4", "actionType": "drop"},
                ],
                'task_assigned': False,
                'task_completed': False,
                'agent_id': None,
            }
        ]
        mock_task_management = MagicMock()
        mock_task_management.task_list = task_list
        instance.task_management = mock_task_management

        return instance


@pytest.fixture(scope="module")
def task_t1():
    return {
        'task_id': 'T1',
        'stations': [
            {"nodeId": "N2", "actionType": "pick"},
            {"nodeId": "N3", "actionType": "process", "processingTime": 1.0},
            {"nodeId": "N4", "actionType": "drop"},
        ],
        'task_assigned': False,
        'task_completed': False,
        'agent_id': None,
    }


@pytest.fixture(scope="module")
def path_result(fm, task_t1):
    """Cache the result of build_path_for_task(T1, N5) to reuse across tests."""
    result = fm.build_path_for_task(task_t1, "N5")
    if result is None:
        pytest.skip("build_path_for_task() returned None — not yet implemented.")
    return result


# ── build_path_for_task ───────────────────────────────────────────────────────

def test_build_path_not_empty(path_result):
    path_nodes, path_edges = path_result
    assert path_nodes, "build_path_for_task must return a non-empty path_nodes list"
    assert path_edges is not None, "build_path_for_task must return a path_edges list"


def test_build_path_starts_at_start_node(path_result):
    path_nodes, _ = path_result
    assert path_nodes[0] == "N5", \
        f"Path must start at the given start_node 'N5', got '{path_nodes[0]}'"


def test_build_path_visits_all_stations(path_result, task_t1):
    path_nodes, _ = path_result
    for station in task_t1['stations']:
        assert station['nodeId'] in path_nodes, \
            f"Station node '{station['nodeId']}' must appear in the path. " \
            f"Got path: {path_nodes}"


def test_build_path_ends_at_dwelling_node(path_result, graph):
    path_nodes, _ = path_result
    assert path_nodes[-1] in graph.dwelling_nodes, \
        f"Path must end at a dwelling node {graph.dwelling_nodes}, " \
        f"got '{path_nodes[-1]}'"


def test_build_path_nodes_connected(path_result, graph):
    path_nodes, path_edges = path_result
    for i in range(len(path_nodes) - 1):
        edge = graph.get_connected_edge(path_nodes[i], path_nodes[i + 1])
        assert edge is not None, \
            f"No edge between consecutive nodes '{path_nodes[i]}' and " \
            f"'{path_nodes[i + 1]}' in the graph"


def test_build_path_edges_length(path_result):
    path_nodes, path_edges = path_result
    assert len(path_edges) == len(path_nodes) - 1, \
        f"len(path_edges) must equal len(path_nodes) - 1. " \
        f"Got {len(path_nodes)} nodes and {len(path_edges)} edges."


# ── build_order_nodes ─────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def order_nodes(fm, task_t1, path_result):
    path_nodes, _ = path_result
    result = fm.build_order_nodes(path_nodes, task_t1)
    if result is None:
        pytest.skip("build_order_nodes() returned None — not yet implemented.")
    return result


def test_order_nodes_not_empty(order_nodes, path_result):
    path_nodes, _ = path_result
    assert len(order_nodes) == len(path_nodes), \
        f"build_order_nodes must return one dict per path node. " \
        f"Expected {len(path_nodes)}, got {len(order_nodes)}."


def test_order_nodes_required_fields(order_nodes):
    for node in order_nodes:
        for field in ('nodeId', 'x', 'y', 'theta', 'actions'):
            assert field in node, \
                f"Node dict missing required field '{field}': {node}"


def test_order_nodes_pick_station_has_pick_action(order_nodes):
    """The N2 pick station must have a 'pick' action."""
    pick_nodes = [n for n in order_nodes if n['nodeId'] == 'N2'
                  and any(a['actionType'] == 'pick' for a in n['actions'])]
    assert pick_nodes, \
        "Node N2 (pick station) must have a 'pick' action in build_order_nodes output. " \
        f"N2 entries: {[n for n in order_nodes if n['nodeId'] == 'N2']}"


def test_order_nodes_drop_station_has_drop_action(order_nodes):
    """The N4 drop station must have a 'drop' action."""
    drop_nodes = [n for n in order_nodes if n['nodeId'] == 'N4'
                  and any(a['actionType'] == 'drop' for a in n['actions'])]
    assert drop_nodes, \
        "Node N4 (drop station) must have a 'drop' action in build_order_nodes output. " \
        f"N4 entries: {[n for n in order_nodes if n['nodeId'] == 'N4']}"


def test_order_nodes_process_station_has_process_action(order_nodes, task_t1):
    """The N3 process station must have a 'process' action with processingTime."""
    process_nodes = [n for n in order_nodes if n['nodeId'] == 'N3'
                     and any(a['actionType'] == 'process' for a in n['actions'])]
    assert process_nodes, \
        "Node N3 (process station) must have a 'process' action in build_order_nodes output. " \
        f"N3 entries: {[n for n in order_nodes if n['nodeId'] == 'N3']}"
    proc_action = next(a for a in process_nodes[0]['actions'] if a['actionType'] == 'process')
    expected_time = task_t1['stations'][1]['processingTime']
    assert abs(proc_action.get('processingTime', -1) - expected_time) < 0.001, \
        f"process action must carry processingTime={expected_time}, " \
        f"got {proc_action.get('processingTime')}"


def test_order_nodes_init_fine_pos_before_pick(order_nodes, path_result):
    """The node immediately before N2 (pick) must have an init_fine_positioning action."""
    path_nodes, _ = path_result
    # Find the first occurrence of N2 in the path.
    pick_idx = next((i for i, n in enumerate(path_nodes) if n == 'N2'), None)
    if pick_idx is None or pick_idx == 0:
        pytest.skip("N2 not found in path or is the first node — cannot check preceding node")
    preceding_node_id = path_nodes[pick_idx - 1]
    # Find that node in order_nodes.
    preceding_entry = next(
        (n for n in order_nodes if n['nodeId'] == preceding_node_id
         and order_nodes.index(n) == pick_idx - 1),
        None
    )
    if preceding_entry is None:
        preceding_entry = order_nodes[pick_idx - 1]
    assert any(a['actionType'] == 'init_fine_positioning' for a in preceding_entry['actions']), \
        f"The node before N2 (pick) is '{preceding_node_id}' and must have an " \
        f"'init_fine_positioning' action. Got actions: {preceding_entry['actions']}"


def test_order_nodes_init_fine_pos_before_drop(order_nodes, path_result):
    """The node immediately before N4 (drop) must have an init_fine_positioning action."""
    path_nodes, _ = path_result
    # Find the occurrence of N4 in the path (it should be after N3).
    drop_idx = next((i for i, n in enumerate(path_nodes) if n == 'N4'), None)
    if drop_idx is None or drop_idx == 0:
        pytest.skip("N4 not found in path or is the first node")
    preceding_entry = order_nodes[drop_idx - 1]
    assert any(a['actionType'] == 'init_fine_positioning' for a in preceding_entry['actions']), \
        f"The node before N4 (drop) is '{path_nodes[drop_idx - 1]}' and must have an " \
        f"'init_fine_positioning' action. Got actions: {preceding_entry['actions']}"


def test_order_nodes_positions_match_graph(order_nodes, graph):
    """x, y in each node dict must match the graph node position."""
    for node in order_nodes:
        expected_x, expected_y = graph.nodes[node['nodeId']]['pos']
        assert abs(node['x'] - expected_x) < 0.001, \
            f"Node {node['nodeId']}: x={node['x']} but graph has x={expected_x}"
        assert abs(node['y'] - expected_y) < 0.001, \
            f"Node {node['nodeId']}: y={node['y']} but graph has y={expected_y}"


# ── build_order_edges ─────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def order_edges(fm, path_result):
    path_nodes, path_edges = path_result
    result = fm.build_order_edges(path_nodes, path_edges)
    if result is None:
        pytest.skip("build_order_edges() returned None — not yet implemented.")
    return result


def test_order_edges_length(order_edges, path_result):
    _, path_edges = path_result
    assert len(order_edges) == len(path_edges), \
        f"build_order_edges must return one dict per path edge. " \
        f"Expected {len(path_edges)}, got {len(order_edges)}."


def test_order_edges_required_fields(order_edges):
    for edge in order_edges:
        for field in ('edgeId', 'startNodeId', 'endNodeId', 'actions'):
            assert field in edge, \
                f"Edge dict missing required field '{field}': {edge}"


def test_order_edges_continuity(order_edges, path_result):
    """Each edge must connect consecutive nodes: edge[i] goes from path[i] to path[i+1]."""
    path_nodes, _ = path_result
    for i, edge in enumerate(order_edges):
        assert edge['startNodeId'] == path_nodes[i], \
            f"Edge {i}: startNodeId='{edge['startNodeId']}' but expected '{path_nodes[i]}'"
        assert edge['endNodeId'] == path_nodes[i + 1], \
            f"Edge {i}: endNodeId='{edge['endNodeId']}' but expected '{path_nodes[i + 1]}'"


def test_order_edges_actions_are_lists(order_edges):
    for edge in order_edges:
        assert isinstance(edge['actions'], list), \
            f"Edge 'actions' must be a list, got {type(edge['actions'])}: {edge}"
