import os
import sys
import time
import json
import logging
import platform
import subprocess
from typing import Dict

# Base directory of this script (= fleet_management/). Works regardless of the working directory.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

sys.path.insert(0, os.path.join(_SCRIPT_DIR, "src"))

from fleet_management.graph import Graph
from fleet_management.agents import Agents
from fleet_management.task_management import TaskManagement
from fleet_management.task_assignment import TaskAssignment
from fleet_management.fleet_management import FleetManagement


def setup_logging(log_file_path: str) -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        filename=log_file_path,
        filemode='w'
    )
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(formatter)
    logging.getLogger('').addHandler(console_handler)
    logging.getLogger('matplotlib').setLevel(logging.WARNING)
    return logging.getLogger(__name__)


class ConfigManager:
    """Handles loading and managing configurations."""
    def __init__(self, config_paths: Dict[str, str]):
        self.config_data = self._load_json(config_paths['config'])
        self.lif_data = self._load_json(config_paths['lif'])
        self.agents_initialization_data = self._load_json(config_paths['agents_initialization'])
        self.transportation_tasks_data = self._load_json(config_paths['transportation_tasks'])

    @staticmethod
    def _load_json(path: str) -> dict:
        try:
            with open(path, 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            logging.error(f"File not found: {path}")
            sys.exit(1)
        except json.JSONDecodeError:
            logging.error(f"Invalid JSON format in file: {path}")
            sys.exit(1)


def get_agent_simulation_path() -> str:
    exe_name = "agent_simulation.exe" if platform.system() == "Windows" else "agent_simulation"
    return os.path.join(_SCRIPT_DIR, "src", "mobile_robot_simulation", "dist", exe_name)


def run_simulation(config_manager: ConfigManager, logging: logging.Logger):
    # Start the agent simulation executable.
    agent_simulation_path = get_agent_simulation_path()
    if not os.path.isfile(agent_simulation_path):
        logging.error(
            f"Agent simulation executable not found: {agent_simulation_path}. "
            "Build it first with src/mobile_robot_simulation/build_executable.py."
        )
        sys.exit(1)

    try:
        proc = subprocess.Popen([agent_simulation_path])
        logging.info("Agent simulation executable started. Waiting for it to initialize...")
        # TODO: Adjust sleep time if the agent simulation takes longer to start on your system.
        time.sleep(6)
    except OSError as e:
        logging.error(f"Failed to start agent simulation: {e}")
        sys.exit(1)

    # Record simulation start time.
    simulation_start_time = time.time()

    # Create the graph.
    graph = Graph(lif_data=config_manager.lif_data)

    # Create the Agents object. For each agent, a digital twin agent object is created.
    agents = Agents(config_data=config_manager.config_data, graph=graph,
                    agents_initialization_data=config_manager.agents_initialization_data,
                    logging=logging, simulation_start_time=simulation_start_time)

    # Create the fleet management objects.
    task_management = TaskManagement(graph=graph, agents=agents,
                                     transportation_tasks_data=config_manager.transportation_tasks_data,
                                     simulation_start_time=simulation_start_time)
    task_assignment = TaskAssignment(graph=graph, agents=agents, task_management=task_management,
                                     simulation_start_time=simulation_start_time)
    fleet_management = FleetManagement(config_data=config_manager.config_data, graph=graph, agents=agents,
                                       task_management=task_management,
                                       simulation_start_time=simulation_start_time)

    # Start the task completion monitoring thread.
    task_management.check_completion_thread.start()

    # Run until the configured simulation time expires, then terminate the executable.
    try:
        time.sleep(config_manager.config_data["simulation_run_time"])
    except KeyboardInterrupt:
        logging.info("Simulation stopped by user (Ctrl+C).")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def main():
    logging = setup_logging(os.path.join(_SCRIPT_DIR, "data", "output_files", "logging_file.log"))

    config_paths = {
        "config":               os.path.join(_SCRIPT_DIR, "data", "input_files", "config_file.json"),
        "lif":                  os.path.join(_SCRIPT_DIR, "data", "input_files", "lif_file.json"),
        "agents_initialization": os.path.join(_SCRIPT_DIR, "data", "input_files", "agentsInitialization_file.json"),
        "transportation_tasks": os.path.join(_SCRIPT_DIR, "data", "input_files", "transportationTasks_file.json"),
    }

    config_manager = ConfigManager(config_paths)
    run_simulation(config_manager, logging)

if __name__ == "__main__":
    main()
