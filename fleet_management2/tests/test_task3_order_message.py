"""
Task 3 Validation — generate_order_message()
Run from the workspace root:  pytest fleet_management/tests/test_task3_order_message.py -v

Tests that generate_order_message() produces a valid VDA 5050 order message
from a manually provided nodes/edges input list.

This test does NOT require Task 4 (Graph) or Task 6 (A*) to be implemented.
The path data is hardcoded here and matches exactly the reference order message
in data/input_files/orderMessage_Example.json (the Task 2 reference file),
exactly as students are expected to hardcode it manually in fleet_manager()
for Task 3.

The robot follows the structure:
    dwelling (N5) -> N1 -> N7 (init_fine_pos) -> pick (N2) -> N7 -> N8
    -> process (N3, 1.0s) -> N13 -> N12 (init_fine_pos) -> drop (N4)
    -> N12 -> N13 -> dwelling (N14)
"""
import sys
import os
import json
import pytest
from unittest.mock import patch, MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data', 'input_files')

# paho-mqtt is only needed at runtime (broker connection). Stub it out so the
# tests run in any environment where paho is not installed.
for _mod in ('paho', 'paho.mqtt', 'paho.mqtt.client'):
    sys.modules.setdefault(_mod, MagicMock())


# ── Hardcoded path from orderMessage_Example.json ─────────────────────────────
#
# Task 3 asks students to manually fill path_nodes and path_edges in
# fleet_manager().  The data below is taken verbatim from
# data/input_files/orderMessage_Example.json (the Task 2 reference file) so that
# this test validates the exact same journey students inspect in Task 2.
#
# Journey: N5 → N1 → N7(init_fine_pos) → N2(pick) → N7 → N8
#          → N3(process, 1.0s) → N13 → N12(init_fine_pos) → N4(drop)
#          → N12 → N13 → N14(dwelling)
#
# Action rules (same rules students must apply in fleet_manager()):
#   - Node BEFORE a TRANSFER station (pick/drop): init_fine_positioning
#   - TRANSFER station node (pick/drop):          pick / drop action
#   - PROCESS station node:                       process action + processingTime

NODES_INPUT = [
    # dwelling — start position, no action
    {"nodeId": "N5",  "x": 1.05, "y": 0.90, "theta": None,  "actions": []},
    # intermediate node
    {"nodeId": "N1",  "x": 2.46, "y": 0.79, "theta": -1.57, "actions": []},
    # node immediately before N2 (TRANSFER pick) → init_fine_positioning
    {"nodeId": "N7",  "x": 2.46, "y": 2.25, "theta": None,
     "actions": [
         {"actionType": "init_fine_positioning",
          "actionId": "81276cdd-0c17-4c7b-a2b7-9150d3d6c80f", "blockingType": "HARD"}
     ]},
    # TRANSFER station → pick action
    {"nodeId": "N2",  "x": 2.46, "y": 3.70, "theta": 1.57,
     "actions": [
         {"actionType": "pick",
          "actionId": "abaa06db-ffb5-4eef-8de3-6cf74d722cdf", "blockingType": "HARD"}
     ]},
    # backtrack through N7 after pick
    {"nodeId": "N7",  "x": 2.46, "y": 2.25, "theta": None,  "actions": []},
    # intermediate node
    {"nodeId": "N8",  "x": 4.00, "y": 2.25, "theta": None,  "actions": []},
    # PROCESS station → process action with processingTime
    {"nodeId": "N3",  "x": 5.45, "y": 2.25, "theta": None,
     "actions": [
         {"actionType": "process",
          "actionId": "22817b07-4a1c-4698-ad73-78539dea88ab",
          "blockingType": "HARD", "processingTime": 1.0}
     ]},
    # intermediate node
    {"nodeId": "N13", "x": 5.55, "y": 3.50, "theta": None,  "actions": []},
    # node immediately before N4 (TRANSFER drop) → init_fine_positioning
    {"nodeId": "N12", "x": 4.00, "y": 3.50, "theta": None,
     "actions": [
         {"actionType": "init_fine_positioning",
          "actionId": "897b0d7d-d395-400e-b2a9-2f82738502a8", "blockingType": "HARD"}
     ]},
    # TRANSFER station → drop action
    {"nodeId": "N4",  "x": 5.55, "y": 4.75, "theta": 0.0,
     "actions": [
         {"actionType": "drop",
          "actionId": "844538f5-0c63-4ddc-9b76-7bed887b96b5", "blockingType": "HARD"}
     ]},
    # return path to dwelling N14
    {"nodeId": "N12", "x": 4.00, "y": 3.50, "theta": None,  "actions": []},
    {"nodeId": "N13", "x": 5.55, "y": 3.50, "theta": None,  "actions": []},
    # dwelling — end position, no action
    {"nodeId": "N14", "x": 6.80, "y": 3.50, "theta": None,  "actions": []},
]

EDGES_INPUT = [
    {"edgeId": "E21", "startNodeId": "N5",  "endNodeId": "N1",  "actions": []},
    {"edgeId": "E1",  "startNodeId": "N1",  "endNodeId": "N7",  "actions": []},
    {"edgeId": "E2",  "startNodeId": "N7",  "endNodeId": "N2",  "actions": []},
    {"edgeId": "E2",  "startNodeId": "N2",  "endNodeId": "N7",  "actions": []},
    {"edgeId": "E3",  "startNodeId": "N7",  "endNodeId": "N8",  "actions": []},
    {"edgeId": "E5",  "startNodeId": "N8",  "endNodeId": "N3",  "actions": []},
    {"edgeId": "E17", "startNodeId": "N3",  "endNodeId": "N13", "actions": []},
    {"edgeId": "E12", "startNodeId": "N13", "endNodeId": "N12", "actions": []},
    {"edgeId": "E19", "startNodeId": "N12", "endNodeId": "N4",  "actions": []},
    {"edgeId": "E19", "startNodeId": "N4",  "endNodeId": "N12", "actions": []},
    {"edgeId": "E12", "startNodeId": "N12", "endNodeId": "N13", "actions": []},
    {"edgeId": "E16", "startNodeId": "N13", "endNodeId": "N14", "actions": []},
]


# ── Fixture: call generate_order_message() and capture the published dict ─────

@pytest.fixture(scope="module")
def config_data():
    with open(os.path.join(_DATA_DIR, "config_file.json")) as f:
        return json.load(f)

@pytest.fixture(scope="module")
def published_message(config_data):
    """
    Calls generate_order_message() with the hardcoded path above and
    captures the dict that would be published to the MQTT broker.
    No broker connection is needed — publish is mocked.
    """
    captured = {}

    from vda5050_interface.interfaces.order_interface import OrderInterface

    with patch("vda5050_interface.interfaces.order_interface.MQTTPublisher") as MockMQTT:
        mock_publisher = MagicMock()
        MockMQTT.return_value = mock_publisher
        mock_publisher.publish.side_effect = lambda msg, qos: captured.update({"msg": msg})

        mock_agent = MagicMock()
        mock_agent.agents.order_header_id = 1
        mock_agent.agentId = "mouse001"

        oi = OrderInterface(config_data=config_data, logging=MagicMock(),
                            order_topic="test/topic", agentId="mouse001")
        oi.generate_order_message(agent=mock_agent, orderId="1", order_updateId=0,
                                  nodes=NODES_INPUT, edges=EDGES_INPUT)

    if not captured:
        pytest.skip("generate_order_message() never called publish() — not yet implemented.")
    return captured["msg"]


# ── Stub-detection test ──────────────────────────────────────────────────────

def test_message_built_dynamically(config_data):
    """
    Passes only when generate_order_message() builds the message from its
    arguments rather than loading orderMessage_Example.json.

    Strategy: call with orderId='SENTINEL' and order_updateId=42 — values
    that do not appear in the example file.  A dynamic implementation will
    echo them back; the stub will not.
    """
    captured = {}
    from vda5050_interface.interfaces.order_interface import OrderInterface

    with patch("vda5050_interface.interfaces.order_interface.MQTTPublisher") as MockMQTT:
        mock_publisher = MagicMock()
        MockMQTT.return_value = mock_publisher
        mock_publisher.publish.side_effect = lambda msg, qos: captured.update({"msg": msg})

        mock_agent = MagicMock()
        oi = OrderInterface(config_data=config_data, logging=MagicMock(),
                            order_topic="test/topic", agentId="mouse001")
        oi.generate_order_message(agent=mock_agent, orderId="SENTINEL",
                                  order_updateId=42, nodes=[], edges=[])

    assert captured, "generate_order_message() never called publish() — not yet implemented."
    msg = captured["msg"]
    assert msg.get("orderId") == "SENTINEL" and msg.get("orderUpdateId") == 42, (
        f"generate_order_message() returned orderId={msg.get('orderId')!r}, "
        f"orderUpdateId={msg.get('orderUpdateId')!r} — expected 'SENTINEL' and 42. "
        "The function is still publishing the hardcoded example file instead of building "
        "the message from its arguments. Implement generate_order_message() in order_interface.py."
    )


# ── Required top-level fields ────────────────────────────────────────────────

REQUIRED_FIELDS = ["headerId", "timestamp", "version", "manufacturer",
                   "serialNumber", "orderId", "orderUpdateId", "nodes", "edges"]

def test_required_fields_present(published_message):
    for field in REQUIRED_FIELDS:
        assert field in published_message, f"Order message missing required field: '{field}'"

def test_nodes_list(published_message):
    assert isinstance(published_message["nodes"], list), "'nodes' must be a list"
    assert len(published_message["nodes"]) == len(NODES_INPUT), \
        f"Expected {len(NODES_INPUT)} nodes (matching input), got {len(published_message['nodes'])}"

def test_edges_list(published_message):
    assert isinstance(published_message["edges"], list), "'edges' must be a list"
    assert len(published_message["edges"]) == len(EDGES_INPUT), \
        f"Expected {len(EDGES_INPUT)} edges (matching input), got {len(published_message['edges'])}"


# ── sequenceId ordering (VDA 5050 rule) ─────────────────────────────────────

def test_first_node_sequenceId_is_zero(published_message):
    first_seq = published_message["nodes"][0]["sequenceId"]
    assert first_seq == 0, \
        f"First node sequenceId must be 0 (current position), got {first_seq}"

def test_node_sequenceIds_are_even(published_message):
    for node in published_message["nodes"]:
        assert node["sequenceId"] % 2 == 0, \
            f"Node {node['nodeId']} sequenceId must be even (0, 2, 4…), got {node['sequenceId']}"

def test_edge_sequenceIds_are_odd(published_message):
    for edge in published_message["edges"]:
        assert edge["sequenceId"] % 2 == 1, \
            f"Edge {edge['edgeId']} sequenceId must be odd (1, 3, 5…), got {edge['sequenceId']}"

def test_sequenceIds_alternating(published_message):
    """Node and edge sequenceIds must interleave: 0, 1, 2, 3, 4, …"""
    all_seq = sorted(
        [(n["sequenceId"], "node") for n in published_message["nodes"]] +
        [(e["sequenceId"], "edge") for e in published_message["edges"]]
    )
    for i, (seq, kind) in enumerate(all_seq):
        assert seq == i, \
            f"sequenceId gap: expected {i}, got {seq} ({kind}). Check alternating node/edge sequenceIds."

def test_n_edges_equals_n_nodes_minus_one(published_message):
    n = len(published_message["nodes"])
    e = len(published_message["edges"])
    assert e == n - 1, f"Expected {n-1} edges for {n} nodes, got {e}"


# ── Node fields ──────────────────────────────────────────────────────────────

def test_node_fields(published_message):
    for node in published_message["nodes"]:
        assert "nodeId" in node, f"Node missing 'nodeId': {node}"
        assert "sequenceId" in node, f"Node missing 'sequenceId': {node}"
        assert "released" in node, f"Node missing 'released': {node}"
        assert "nodePosition" in node, f"Node missing 'nodePosition': {node}"
        assert "actions" in node, f"Node missing 'actions': {node}"
        pos = node["nodePosition"]
        assert "x" in pos and "y" in pos and "mapId" in pos, \
            f"nodePosition missing x/y/mapId: {pos}"

def test_theta_only_when_not_none(published_message):
    """theta must only appear in nodePosition when it is not None."""
    for node in published_message["nodes"]:
        pos = node["nodePosition"]
        if "theta" in pos:
            assert pos["theta"] is not None, \
                f"Node {node['nodeId']}: theta is in nodePosition but is None — omit it instead"

def test_actions_preserved(published_message):
    """Actions from the input must appear unchanged in the published message."""
    # N2 is the pick station (first occurrence, index 3 in the input)
    n2_nodes = [n for n in published_message["nodes"] if n["nodeId"] == "N2"]
    assert n2_nodes, "Node N2 (pick station) must be in the published message"
    pick_actions = [a for a in n2_nodes[0]["actions"] if a.get("actionType") == "pick"]
    assert pick_actions, "Node N2 must have a 'pick' action in the published message"

    # N4 is the drop station
    n4_nodes = [n for n in published_message["nodes"] if n["nodeId"] == "N4"]
    assert n4_nodes, "Node N4 (drop station) must be in the published message"
    drop_actions = [a for a in n4_nodes[0]["actions"] if a.get("actionType") == "drop"]
    assert drop_actions, "Node N4 must have a 'drop' action in the published message"


# ── Edge fields ──────────────────────────────────────────────────────────────

def test_edge_fields(published_message):
    for edge in published_message["edges"]:
        assert "edgeId" in edge, f"Edge missing 'edgeId': {edge}"
        assert "sequenceId" in edge, f"Edge missing 'sequenceId': {edge}"
        assert "released" in edge, f"Edge missing 'released': {edge}"
        assert "startNodeId" in edge, f"Edge missing 'startNodeId': {edge}"
        assert "endNodeId" in edge, f"Edge missing 'endNodeId': {edge}"
        assert "actions" in edge, f"Edge missing 'actions': {edge}"

def test_edge_node_continuity(published_message):
    """Each edge must connect consecutive nodes."""
    nodes = published_message["nodes"]
    edges = published_message["edges"]
    for i, edge in enumerate(edges):
        assert edge["startNodeId"] == nodes[i]["nodeId"], \
            f"Edge {i} startNodeId {edge['startNodeId']} != node {i} id {nodes[i]['nodeId']}"
        assert edge["endNodeId"] == nodes[i + 1]["nodeId"], \
            f"Edge {i} endNodeId {edge['endNodeId']} != node {i+1} id {nodes[i+1]['nodeId']}"
