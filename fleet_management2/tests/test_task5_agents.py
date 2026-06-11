"""
Task 5 Validation — Agent digital twin
Run from the workspace root:  pytest fleet_management/tests/test_task5_agents.py -v

Tests that:
  1. get_agents() creates one Agent per entry in agentsInitialization_file.json
     (not just agents[0]).
  2. The Agent class has 'current_node' and 'current_task' attributes.
  3. All agent attributes have correct initial values.

No MQTT broker needed — broker calls are mocked.
"""
import sys
import os
import json
import time
import pytest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data', 'input_files')

# Stub paho-mqtt so tests run without a broker connection
for _mod in ('paho', 'paho.mqtt', 'paho.mqtt.client'):
    sys.modules.setdefault(_mod, MagicMock())


@pytest.fixture(scope="module")
def agents_init_data():
    with open(os.path.join(_DATA_DIR, "agentsInitialization_file.json")) as f:
        return json.load(f)


@pytest.fixture(scope="module")
def agents_obj(agents_init_data):
    """Creates an Agents container with MQTT connections mocked out."""
    with patch("vda5050_interface.mqtt_clients.mqtt_subscriber.MQTTSubscriber"), \
         patch("vda5050_interface.mqtt_clients.mqtt_publisher.MQTTPublisher"), \
         patch("vda5050_interface.interfaces.order_interface.OrderInterface"):

        from fleet_management.agents import Agents

        with open(os.path.join(_DATA_DIR, "config_file.json")) as f:
            config_data = json.load(f)

        obj = Agents(
            config_data=config_data,
            graph=MagicMock(),
            agents_initialization_data=agents_init_data,
            logging=MagicMock(),
            simulation_start_time=time.time(),
        )
        return obj


# ── Agent count ───────────────────────────────────────────────────────────────

def test_agents_count_matches_init_file(agents_obj, agents_init_data):
    """get_agents() must create one Agent per entry in agentsInitialization_file.json,
    not only agents_initialization_data['agents'][0]."""
    expected = len(agents_init_data['agents'])
    actual = len(agents_obj.agents)
    assert actual == expected, (
        f"Expected {expected} agent(s) — one per entry in agentsInitialization_file.json — "
        f"got {actual}. "
        f"Make sure get_agents() loops over all entries, not just [0]."
    )


# ── Required new attributes ───────────────────────────────────────────────────

def test_agent_has_current_node(agents_obj):
    """Agent must expose a 'current_node' attribute (used as A* start in Task 6)."""
    agent = agents_obj.agents[0]
    assert hasattr(agent, 'current_node'), (
        "Agent must have a 'current_node' attribute. "
        "Add 'self.current_node = ...' in Agent.__init__()."
    )


def test_agent_has_current_task(agents_obj):
    """Agent must expose a 'current_task' attribute (set by task assignment in Task 8)."""
    agent = agents_obj.agents[0]
    assert hasattr(agent, 'current_task'), (
        "Agent must have a 'current_task' attribute. "
        "Add 'self.current_task = None' in Agent.__init__()."
    )


# ── Initial values ────────────────────────────────────────────────────────────

def test_agent_state_initially_idle(agents_obj):
    assert agents_obj.agents[0].agent_state == "IDLE", (
        f"agent_state must be 'IDLE' on initialization, "
        f"got '{agents_obj.agents[0].agent_state}'"
    )


def test_current_task_initially_none(agents_obj):
    assert agents_obj.agents[0].current_task is None, (
        f"current_task must be None on initialization, "
        f"got {agents_obj.agents[0].current_task!r}"
    )


def test_loaded_initially_false(agents_obj):
    assert agents_obj.agents[0].loaded is False, (
        f"'loaded' must be False on initialization, "
        f"got {agents_obj.agents[0].loaded!r}"
    )


def test_agent_id_matches_init_data(agents_obj, agents_init_data):
    expected_id = agents_init_data['agents'][0]['agentId']
    actual_id = agents_obj.agents[0].agentId
    assert actual_id == expected_id, (
        f"agents[0].agentId should be '{expected_id}', got '{actual_id}'"
    )


def test_current_node_is_string_or_none(agents_obj):
    """current_node must be a string (node ID) or None — never a dict or other type."""
    node = agents_obj.agents[0].current_node
    assert node is None or isinstance(node, str), (
        f"current_node must be a str or None, "
        f"got {type(node).__name__!r}: {node!r}"
    )
