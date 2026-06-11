"""
Task 10 Validation — Full automation (fleet_manager loop + daemon thread)
Run from the workspace root:  pytest fleet_management/tests/test_task10_fleet_manager.py -v

Tests that:
  1. FleetManagement.__init__() launches fleet_manager() in a daemon thread
     (not as a blocking call).
  2. fleet_manager() assigns all tasks sequentially once agents become idle.
  3. fleet_manager() does NOT assign a task to a busy (EXECUTING) agent.

No MQTT broker needed — all I/O is mocked.
Requires Tasks 5 and 6 to be implemented first.
"""
import sys
import os
import json
import time
import threading
import pytest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
_DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data', 'input_files')

# Stub paho-mqtt so tests run without a broker connection
for _mod in ('paho', 'paho.mqtt', 'paho.mqtt.client'):
    sys.modules.setdefault(_mod, MagicMock())


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_task_list():
    return [
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


@pytest.fixture
def mock_setup():
    """
    Creates a FleetManagement instance with all MQTT/path-planning mocked.
    The internal helper methods (build_path_for_task, build_order_nodes,
    build_order_edges) are replaced with MagicMocks so fleet_manager() can run
    without a real graph or MQTT broker.
    """
    with patch("vda5050_interface.mqtt_clients.mqtt_subscriber.MQTTSubscriber"), \
         patch("vda5050_interface.mqtt_clients.mqtt_publisher.MQTTPublisher"), \
         patch("vda5050_interface.interfaces.order_interface.OrderInterface"):

        from fleet_management.fleet_management import FleetManagement

        mock_agent = MagicMock()
        mock_agent.agentId = "mouse001"
        mock_agent.agent_state = "IDLE"
        mock_agent.current_node = "N5"
        mock_agent.current_task = None
        mock_agent.order_interface = MagicMock()

        mock_agents = MagicMock()
        mock_agents.agents = [mock_agent]
        mock_agents.order_header_id = 1

        task_list = _make_task_list()
        mock_task_management = MagicMock()
        mock_task_management.task_list = task_list

        mock_graph = MagicMock()
        mock_graph.nodes = {
            "N5": {"pos": (1.05, 0.90), "vehicleTypeNodeProperties": []},
            "N4": {"pos": (5.55, 4.75), "vehicleTypeNodeProperties": []},
            "N2": {"pos": (2.46, 3.70), "vehicleTypeNodeProperties": []},
        }
        mock_graph.dwelling_nodes = ["N5"]

        with open(os.path.join(_DATA_DIR, "config_file.json")) as f:
            config_data = json.load(f)

        # Bypass __init__ so no thread is started before we are ready
        with patch.object(FleetManagement, '__init__', lambda *a, **kw: None):
            fm = FleetManagement.__new__(FleetManagement)

        fm.config_data = config_data
        fm.graph = mock_graph
        fm.agents = mock_agents
        fm.task_management = mock_task_management
        fm.simulation_start_time = time.time()

        # Stub path-planning internals
        fm.path_planning = MagicMock()
        fm.path_planning.astar_search.return_value = (["N5", "N4"], ["E21"])
        fm.build_path_for_task = MagicMock(return_value=(["N5", "N4"], ["E21"]))
        fm.build_order_nodes = MagicMock(return_value=[])
        fm.build_order_edges = MagicMock(return_value=[])

        yield fm, mock_agent, task_list


# ── Daemon thread ─────────────────────────────────────────────────────────────

def test_init_starts_daemon_thread():
    """
    FleetManagement.__init__() must launch fleet_manager() in a daemon thread,
    not call it directly (which would block).

    Implementation hint:
        Replace `self.fleet_manager()` with:
            import threading
            threading.Thread(target=self.fleet_manager, daemon=True).start()
    """
    with patch("vda5050_interface.mqtt_clients.mqtt_subscriber.MQTTSubscriber"), \
         patch("vda5050_interface.mqtt_clients.mqtt_publisher.MQTTPublisher"), \
         patch("vda5050_interface.interfaces.order_interface.OrderInterface"):

        from fleet_management.fleet_management import FleetManagement
        from fleet_management.graph import Graph

        with open(os.path.join(_DATA_DIR, "config_file.json")) as f:
            config_data = json.load(f)
        with open(os.path.join(_DATA_DIR, "lif_file.json")) as f:
            lif_data = json.load(f)

        graph = Graph(lif_data=lif_data)

        # Keep agent EXECUTING so fleet_manager loops indefinitely — this lets
        # us detect whether __init__ blocks or returns immediately.
        mock_agent = MagicMock()
        mock_agent.agentId = "mouse001"
        mock_agent.agent_state = "EXECUTING"
        mock_agent.current_node = "N5"
        mock_agent.current_task = None
        mock_agent.order_interface = MagicMock()

        mock_agents = MagicMock()
        mock_agents.agents = [mock_agent]
        mock_agents.order_header_id = 1

        mock_task_management = MagicMock()
        mock_task_management.task_list = _make_task_list()

        threads_before = set(t.ident for t in threading.enumerate())

        fm = FleetManagement(
            config_data=config_data,
            graph=graph,
            agents=mock_agents,
            task_management=mock_task_management,
            simulation_start_time=time.time(),
        )

        # Give the thread a moment to start up
        time.sleep(0.3)

        new_daemon_threads = [
            t for t in threading.enumerate()
            if t.ident not in threads_before and t.daemon
        ]
        assert len(new_daemon_threads) >= 1, (
            "FleetManagement.__init__() must start fleet_manager() as a daemon thread "
            "so the constructor returns immediately. "
            "Replace `self.fleet_manager()` with: "
            "threading.Thread(target=self.fleet_manager, daemon=True).start()"
        )


# ── Automation behaviour ──────────────────────────────────────────────────────

def test_fleet_manager_assigns_all_tasks(mock_setup):
    """
    fleet_manager() must assign ALL tasks when the agent becomes idle between orders.
    The while loop must continue until every task is assigned.
    """
    fm, mock_agent, task_list = mock_setup

    # Fresh state
    mock_agent.agent_state = "IDLE"
    for t in task_list:
        t['task_assigned'] = False
        t['agent_id'] = None

    def simulate_task_done(*args, **kwargs):
        """Asynchronously mimic the robot completing its order after a short delay.

        The side_effect runs *inside* generate_order_message(), before
        fleet_manager() sets agent_state = 'EXECUTING'.  Using a background
        thread lets fleet_manager() set EXECUTING first, and then the 'robot'
        reports back IDLE — matching the real MQTT flow.
        """
        def _complete():
            time.sleep(0.15)
            mock_agent.agent_state = "IDLE"
        threading.Thread(target=_complete, daemon=True).start()

    mock_agent.order_interface.generate_order_message.side_effect = simulate_task_done

    thread = threading.Thread(target=fm.fleet_manager, daemon=True)
    thread.start()
    thread.join(timeout=10.0)

    assigned = [t for t in task_list if t['task_assigned']]
    assert len(assigned) == len(task_list), (
        f"fleet_manager() must assign ALL {len(task_list)} tasks. "
        f"Only {len(assigned)} were assigned. "
        f"Make sure fleet_manager() uses a while loop that continues "
        f"until all tasks are assigned."
    )


def test_fleet_manager_waits_for_idle_agent(mock_setup):
    """
    fleet_manager() must NOT send an order to a busy (EXECUTING) agent.
    It must wait until an agent is IDLE before assigning the next task.

    The test first confirms that fleet_manager() CAN assign when the agent is
    IDLE (prerequisite), then re-runs with a permanently EXECUTING agent to
    verify that no assignment happens.
    """
    fm, mock_agent, task_list = mock_setup

    # ── Step 1: prerequisite — IDLE agent must get a task ────────────────────
    mock_agent.agent_state = "IDLE"
    for t in task_list:
        t['task_assigned'] = False
        t['agent_id'] = None

    def simulate_task_done(*args, **kwargs):
        def _complete():
            time.sleep(0.15)
            mock_agent.agent_state = "IDLE"
        threading.Thread(target=_complete, daemon=True).start()

    mock_agent.order_interface.generate_order_message.side_effect = simulate_task_done

    thread = threading.Thread(target=fm.fleet_manager, daemon=True)
    thread.start()
    thread.join(timeout=10.0)

    assert any(t['task_assigned'] for t in task_list), (
        "Prerequisite failed: fleet_manager() must assign tasks to an IDLE agent. "
        "Fix the fleet_manager() loop before this test can verify the 'wait for IDLE' behaviour."
    )

    # ── Step 2: EXECUTING agent — nothing should be assigned ─────────────────
    mock_agent.agent_state = "EXECUTING"
    mock_agent.order_interface.generate_order_message.side_effect = None
    for t in task_list:
        t['task_assigned'] = False
        t['agent_id'] = None

    thread = threading.Thread(target=fm.fleet_manager, daemon=True)
    thread.start()
    thread.join(timeout=2.0)

    assigned = [t for t in task_list if t['task_assigned']]
    assert len(assigned) == 0, (
        f"fleet_manager() must wait for an idle agent before assigning tasks. "
        f"{len(assigned)} task(s) were incorrectly assigned while the agent was EXECUTING."
    )
