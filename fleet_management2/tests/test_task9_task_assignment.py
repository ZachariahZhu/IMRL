"""
Task 9 Validation — TaskAssignment
Run from the workspace root:  pytest fleet_management/tests/test_task9_task_assignment.py -v

Tests that task_assignment_manager() correctly assigns unassigned tasks to idle agents.
No MQTT broker needed — all components are mocked.
Requires Task 8 to be implemented first (agent_state must be updated by state_callback).
"""
import sys
import os
import time
import threading
import pytest
from unittest.mock import MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Stub paho-mqtt so tests run without a broker connection
for _mod in ('paho', 'paho.mqtt', 'paho.mqtt.client'):
    sys.modules.setdefault(_mod, MagicMock())


@pytest.fixture
def mock_setup():
    """Create minimal mock objects for TaskAssignment without an MQTT broker."""
    from unittest.mock import patch
    with patch("vda5050_interface.mqtt_clients.mqtt_subscriber.MQTTSubscriber"), \
         patch("vda5050_interface.mqtt_clients.mqtt_publisher.MQTTPublisher"), \
         patch("vda5050_interface.interfaces.order_interface.OrderInterface"):

        from fleet_management.task_assignment import TaskAssignment

        mock_agent = MagicMock()
        mock_agent.agentId = "mouse001"
        mock_agent.agent_state = "IDLE"
        mock_agent.current_node = "N5"

        mock_agents = MagicMock()
        mock_agents.agents = [mock_agent]

        mock_task_management = MagicMock()
        mock_task_management.task_list = [
            {
                'task_id': 'T1',
                'stations': [{"nodeId": "N4", "actionType": "pick"},
                              {"nodeId": "N2", "actionType": "drop"}],
                'task_assigned': False,
                'task_completed': False,
                'agent_id': None,
            },
            {
                'task_id': 'T2',
                'stations': [{"nodeId": "N2", "actionType": "pick"},
                              {"nodeId": "N4", "actionType": "drop"}],
                'task_assigned': False,
                'task_completed': False,
                'agent_id': None,
            },
        ]

        mock_graph = MagicMock()
        mock_graph.nodes = {
            "N5": {"pos": (1.05, 0.90)},
            "N4": {"pos": (5.55, 4.75)},
            "N2": {"pos": (2.46, 3.70)},
        }

        ta = TaskAssignment(
            graph=mock_graph,
            agents=mock_agents,
            task_management=mock_task_management,
            simulation_start_time=time.time()
        )
        yield ta, mock_agent, mock_task_management


# ── Assignment behaviour ─────────────────────────────────────────────────────

def test_manager_assigns_task(mock_setup):
    """Running task_assignment_manager() must mark at least one task as assigned."""
    ta, mock_agent, mock_task_management = mock_setup

    # Reset state
    mock_agent.agent_state = "IDLE"
    for t in mock_task_management.task_list:
        t['task_assigned'] = False
        t['agent_id'] = None

    thread = threading.Thread(target=ta.task_assignment_manager, daemon=True)
    thread.start()
    thread.join(timeout=3.0)

    assigned = [t for t in mock_task_management.task_list if t['task_assigned']]
    assert len(assigned) >= 1, \
        "task_assignment_manager() must assign at least one task to an idle agent within 3 s"


def test_manager_sets_agent_id(mock_setup):
    """Assigned tasks must have a non-None agent_id."""
    ta, mock_agent, mock_task_management = mock_setup

    mock_agent.agent_state = "IDLE"
    for t in mock_task_management.task_list:
        t['task_assigned'] = False
        t['agent_id'] = None

    thread = threading.Thread(target=ta.task_assignment_manager, daemon=True)
    thread.start()
    thread.join(timeout=3.0)

    assigned = [t for t in mock_task_management.task_list if t['task_assigned']]
    assert len(assigned) >= 1, \
        "task_assignment_manager() must assign at least one task before checking agent_id"
    for task in assigned:
        assert task['agent_id'] is not None, \
            f"Task {task['task_id']}: agent_id must be set when task is assigned"


def test_manager_marks_agent_executing(mock_setup):
    """The assigned agent's state must change to 'EXECUTING'."""
    ta, mock_agent, mock_task_management = mock_setup

    mock_agent.agent_state = "IDLE"
    for t in mock_task_management.task_list:
        t['task_assigned'] = False
        t['agent_id'] = None

    thread = threading.Thread(target=ta.task_assignment_manager, daemon=True)
    thread.start()
    thread.join(timeout=3.0)

    assigned = [t for t in mock_task_management.task_list if t['task_assigned']]
    assert len(assigned) >= 1, \
        "task_assignment_manager() must assign at least one task before checking agent state"
    assert mock_agent.agent_state == "EXECUTING", \
        "Agent state must be 'EXECUTING' after being assigned a task, " \
        f"got '{mock_agent.agent_state}'"


def test_manager_does_not_assign_busy_agent(mock_setup):
    """A task must not be assigned to an agent that is already EXECUTING.

    First confirms an IDLE agent does get assigned (so we know the manager works),
    then re-runs with a BUSY agent and asserts no new assignment happens.
    """
    ta, mock_agent, mock_task_management = mock_setup

    # Step 1: verify assignment works with an IDLE agent
    mock_agent.agent_state = "IDLE"
    for t in mock_task_management.task_list:
        t['task_assigned'] = False
        t['agent_id'] = None

    thread = threading.Thread(target=ta.task_assignment_manager, daemon=True)
    thread.start()
    thread.join(timeout=3.0)
    assert any(t['task_assigned'] for t in mock_task_management.task_list), \
        "Prerequisite failed: IDLE agent should have received a task"

    # Step 2: reset and run again with a BUSY agent — nothing should be assigned
    mock_agent.agent_state = "EXECUTING"
    for t in mock_task_management.task_list:
        t['task_assigned'] = False
        t['agent_id'] = None

    thread = threading.Thread(target=ta.task_assignment_manager, daemon=True)
    thread.start()
    thread.join(timeout=2.0)

    assigned = [t for t in mock_task_management.task_list if t['task_assigned']]
    assert len(assigned) == 0, \
        "No tasks should be assigned when all agents are EXECUTING"
