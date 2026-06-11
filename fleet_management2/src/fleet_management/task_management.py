import time
import threading
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher


class TaskManagement:
    """Class for managing the transportation tasks of the agents."""

    def __init__(self, graph, agents, transportation_tasks_data, simulation_start_time) -> None:
        self.simulation_start_time = simulation_start_time
        self.graph = graph
        self.agents = agents
        self.transportation_tasks_data = transportation_tasks_data
        self.task_list = self.get_task_list()
        self.mqtt_publisher = MQTTPublisher(config_data=agents.config_data, channel="KIT/IMRL/tasks",
                                            client_id='tasks_publisher', logging=agents.logging)
        self.check_completion_thread = threading.Thread(target=self.check_tasks_completion)

    def get_task_list(self) -> list:
        """
        Build the task list from the transportation tasks data.

        Each task has an ordered list of stations the robot must visit.
        Each station entry contains:
            'nodeId'        : str  - the graph node at which to perform the action
            'actionType'    : str  - 'pick', 'drop', or 'process'
            'processingTime': float (only present for 'process' stations)
        """
        task_list = []
        for task in self.transportation_tasks_data['transportationTasks']:
            task_dict = {
                'task_id': task['transportationTaskId'],
                'stations': task['stations'],
                'task_assigned': False,
                'task_completed': False,
                'agent_id': None
            }
            task_list.append(task_dict)
        return task_list

    def check_tasks_completion(self) -> None:
        """
        Monitor task and agent states until all tasks are completed.
        Publishes the task list to the MQTT broker every second.
        """
        running = True
        while running:
            task_completed = True
            for task in self.task_list:
                if not task['task_completed']:
                    task_completed = False
                    break
            for agent in self.agents.agents:
                if agent.agent_state != "IDLE":
                    task_completed = False
                    break
            self.mqtt_publisher.publish(self.task_list, qos=0)
            if task_completed:
                self.agents.logging.info(
                    f"All tasks are completed at time {round(time.time() - self.simulation_start_time, 2)}.")
                running = False
            time.sleep(1)
