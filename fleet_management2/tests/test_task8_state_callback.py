"""
Task 8 Validation — state_callback()
Run from the workspace root:  pytest fleet_management/tests/test_task8_state_callback.py -v

Tests that state_callback() correctly parses incoming VDA 5050 state messages
and updates the Agent's attributes accordingly.
No MQTT broker needed — the callback is called directly with a mock message.
Requires Tasks 3, 5, and 6 to be implemented first.
"""
import sys
import os
import json
import pytest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data', 'input_files')

# Stub paho-mqtt so tests run without a broker connection
for _mod in ('paho', 'paho.mqtt', 'paho.mqtt.client'):
    sys.modules.setdefault(_mod, MagicMock())


@pytest.fixture(scope="module")
def state_msg_data():
    with open(os.path.join(_DATA_DIR, "stateMessage_Example.json")) as f:
        return json.load(f)

@pytest.fixture(scope="module")
def agent(state_msg_data):
    """Creates a minimal Agent object with MQTT connections mocked out."""
    with patch("vda5050_interface.mqtt_clients.mqtt_subscriber.MQTTSubscriber"), \
         patch("vda5050_interface.mqtt_clients.mqtt_publisher.MQTTPublisher"), \
         patch("vda5050_interface.interfaces.order_interface.OrderInterface"):

        from fleet_management.agents import Agents, Agent

        mock_agents = MagicMock()
        mock_agents.config_data = {"broker_address": "localhost", "broker_port": 1883}
        mock_agents.logging = MagicMock()

        agent = Agent(agents=mock_agents, agentId="mouse001",
                      agent_state_topic="test/state", agent_order_topic="test/order",
                      agent_state="IDLE", logging=MagicMock())
        return agent

def _make_mqtt_msg(payload_dict):
    """Helper: wrap a dict as a mock MQTT message."""
    msg = MagicMock()
    msg.topic = "KIT/IMRL/mouse001/state"
    msg.payload = json.dumps(payload_dict).encode("utf-8")
    return msg


# ── Trigger one state update ─────────────────────────────────────────────────

@pytest.fixture(scope="module")
def agent_after_state(agent, state_msg_data):
    msg = _make_mqtt_msg(state_msg_data)
    agent.state_callback(client=None, userdata=None, msg=msg)
    return agent


# ── agvPosition ──────────────────────────────────────────────────────────────

def test_agvPosition_updated(agent_after_state, state_msg_data):
    assert agent_after_state.agvPosition, "agvPosition must be updated after state_callback"
    expected_x = state_msg_data["agvPosition"]["x"]
    expected_y = state_msg_data["agvPosition"]["y"]
    assert abs(agent_after_state.agvPosition.get("x", -1) - expected_x) < 0.01, \
        f"agvPosition.x should be {expected_x}"
    assert abs(agent_after_state.agvPosition.get("y", -1) - expected_y) < 0.01, \
        f"agvPosition.y should be {expected_y}"


# ── current_node ──────────────────────────────────────────────────────────────

def test_current_node_updated(agent, state_msg_data):
    """state_callback must update current_node from state_msg['lastNodeId'].

    The agent's current_node tracks where it last stopped and is used as the
    A* start node for the next order (Task 6).
    """
    if not hasattr(agent, 'current_node'):
        pytest.skip("current_node attribute not yet implemented (Task 5 required)")
    agent.current_node = "__SENTINEL__"
    msg = _make_mqtt_msg(state_msg_data)
    agent.state_callback(client=None, userdata=None, msg=msg)
    expected = state_msg_data["lastNodeId"]
    assert agent.current_node == expected, (
        f"state_callback must set self.current_node = state_msg['lastNodeId']. "
        f"Expected '{expected}', got {agent.current_node!r}. "
        "Add:  self.current_node = state_msg['lastNodeId']  inside state_callback()."
    )


# ── agent_state ───────────────────────────────────────────────────────────────

def test_agent_state_updated_when_idle(agent):
    """When nodeStates=[], edgeStates=[], and all actionStates are FINISHED
    → the order is complete and agent_state must transition to IDLE.

    The agent is forced into EXECUTING first so this test actually verifies a
    state *transition*, not just that the initial value was never changed.
    """
    agent.agent_state = "EXECUTING"

    idle_state = {
        "headerId": 99, "timestamp": "2025-01-01T00:00:00Z",
        "version": "V2.1.0", "manufacturer": "IFL", "serialNumber": "IFL_mouse001",
        "orderId": "1", "orderUpdateId": 0,
        "lastNodeId": "N5", "lastNodeSequenceId": 0,
        "nodeStates": [], "edgeStates": [],
        "agvPosition": {"positionInitialized": True, "x": 1.05, "y": 0.90,
                        "theta": 0.0, "mapId": "Map_1"},
        "actionStates": [
            {"actionId": "abc-001", "actionType": "pick",  "actionStatus": "FINISHED"},
            {"actionId": "abc-002", "actionType": "drop",  "actionStatus": "FINISHED"},
            {"actionId": "abc-003", "actionType": "process", "actionStatus": "FINISHED"},
        ],
        "batteryState": {"batteryCharge": 80, "charging": False},
        "operatingMode": "AUTOMATIC",
        "errors": [],
        "safetyState": {"eStop": "NONE", "fieldViolation": False}
    }
    msg = _make_mqtt_msg(idle_state)
    agent.state_callback(client=None, userdata=None, msg=msg)
    assert agent.agent_state == "IDLE", \
        f"Agent with nodeStates=[], edgeStates=[], all actions FINISHED " \
        f"should transition to IDLE, got '{agent.agent_state}'"


# ── task_completed ────────────────────────────────────────────────────────────

def test_task_completed_when_order_done(agent):
    """state_callback must set current_task['task_completed'] = True when the
    robot has finished its order (nodeStates=[], edgeStates=[], all actions FINISHED).

    This signals task_assignment_manager() (Task 9) that the agent is free for
    the next task.
    """
    if not hasattr(agent, 'current_task'):
        pytest.skip("current_task attribute not yet implemented (Task 5 required)")
    fake_task = {
        "task_id": "T_cb_test",
        "task_assigned": True,
        "task_completed": False,
        "agent_id": "mouse001",
    }
    agent.current_task = fake_task
    agent.agent_state = "EXECUTING"

    finished_state = {
        "headerId": 200, "timestamp": "2025-01-01T00:00:00Z",
        "version": "V2.1.0", "manufacturer": "IFL", "serialNumber": "IFL_mouse001",
        "orderId": "1", "orderUpdateId": 0,
        "lastNodeId": "N14", "lastNodeSequenceId": 0,
        "nodeStates": [], "edgeStates": [],
        "agvPosition": {"positionInitialized": True, "x": 6.80, "y": 3.50,
                        "theta": 0.0, "mapId": "Map_1"},
        "actionStates": [
            {"actionId": "t-001", "actionType": "pick",    "actionStatus": "FINISHED"},
            {"actionId": "t-002", "actionType": "drop",    "actionStatus": "FINISHED"},
            {"actionId": "t-003", "actionType": "process", "actionStatus": "FINISHED"},
        ],
        "batteryState": {"batteryCharge": 80, "charging": False},
        "operatingMode": "AUTOMATIC",
        "errors": [],
        "safetyState": {"eStop": "NONE", "fieldViolation": False}
    }
    msg = _make_mqtt_msg(finished_state)
    agent.state_callback(client=None, userdata=None, msg=msg)

    assert fake_task["task_completed"] is True, (
        "state_callback must set self.current_task['task_completed'] = True "
        "when nodeStates=[], edgeStates=[], and all actionStates are FINISHED. "
        "Add:  if self.current_task is not None: self.current_task['task_completed'] = True"
    )


def test_agent_stays_executing_when_actions_pending(agent):
    """If some actionStates are not yet FINISHED, the order is still in progress:
    - agent_state must NOT be set to IDLE.
    - agvPosition must still be updated (state_callback always updates position).
    """
    agent.agent_state = "EXECUTING"

    pending_state = {
        "headerId": 100, "timestamp": "2025-01-01T00:00:01Z",
        "version": "V2.1.0", "manufacturer": "IFL", "serialNumber": "IFL_mouse001",
        "orderId": "1", "orderUpdateId": 0,
        "lastNodeId": "N4", "lastNodeSequenceId": 0,
        "nodeStates": [], "edgeStates": [],
        "agvPosition": {"positionInitialized": True, "x": 5.55, "y": 4.75,
                        "theta": 0.0, "mapId": "Map_1"},
        "actionStates": [
            {"actionId": "abc-001", "actionType": "pick",  "actionStatus": "FINISHED"},
            {"actionId": "abc-002", "actionType": "drop",  "actionStatus": "RUNNING"},
        ],
        "batteryState": {"batteryCharge": 80, "charging": False},
        "operatingMode": "AUTOMATIC",
        "errors": [],
        "safetyState": {"eStop": "NONE", "fieldViolation": False}
    }
    msg = _make_mqtt_msg(pending_state)
    agent.state_callback(client=None, userdata=None, msg=msg)

    assert agent.agent_state != "IDLE", \
        "Agent must NOT be IDLE while actionStates still contains non-FINISHED actions"

    expected_x = pending_state["agvPosition"]["x"]  # 5.55
    assert abs(agent.agvPosition.get("x", float("nan")) - expected_x) < 0.01, (
        f"state_callback must update agvPosition even when the order is not yet complete. "
        f"Expected agvPosition.x = {expected_x}, got {agent.agvPosition.get('x')!r}. "
        "Add:  self.agvPosition = state_msg['agvPosition']  unconditionally in state_callback()."
    )
