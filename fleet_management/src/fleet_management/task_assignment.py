import time
import threading
import math


class TaskAssignment:
    """
    Assigns transportation tasks to idle agents.

    Implement task_assignment_manager() so that each idle agent
    automatically receives the next unassigned task from the task list.

    The manager should run in a daemon thread so it does not block the
    main simulation loop.
    """

    def __init__(self, graph, agents, task_management,
                 simulation_start_time) -> None:
        self.graph = graph
        self.agents = agents
        self.task_management = task_management
        self.simulation_start_time = simulation_start_time

        # threading.Thread(target=self.task_assignment_manager, daemon=True).start()

    def task_assignment_manager(self) -> None:
        """
        Continuously assign unassigned tasks to idle agents.

        Task 9 — implementation steps:
            1. Loop until all tasks in self.task_management.task_list are
               assigned:
                   while any(not t['task_assigned']
                             for t in self.task_management.task_list):
            2. Inside the loop, find idle agents:
                   idle_agents = [a for a in self.agents.agents
                                  if a.agent_state == "IDLE"]
            3. Find unassigned tasks:
                   unassigned = [t for t in self.task_management.task_list
                                 if not t['task_assigned']]
            4. For each idle agent (if any unassigned tasks remain):
                   a. Pop the first unassigned task.
                   b. Set task['task_assigned'] = True
                   c. Set task['agent_id']      = agent.agentId
                   d. Set agent.agent_state     = "EXECUTING"
                   e. Set agent.current_task    = task
            5. (Optional) Select the *best* agent per task — e.g. the idle
               agent whose current_node is geographically closest to the
               task's startNodeId.
               Use self.graph.nodes[node_id]['pos'] for coordinates.
            6. Sleep briefly before the next check to avoid busy-waiting:
                   time.sleep(0.5)
        """
        while any(not t['task_assigned'] for t in self.task_management.task_list):
            
            idle_agents = [a for a in self.agents.agents if a.agent_state == "IDLE"]
            unassigned_tasks = [t for t in self.task_management.task_list if not t['task_assigned']]
            
            while idle_agents and unassigned_tasks:
                task = unassigned_tasks.pop(0)
                
                first_station_node = task['stations'][0]['nodeId']
                first_station_pos = self.graph.nodes[first_station_node]['pos']
                
                best_agent = None
                min_dist = float('inf')
                
                for agent in idle_agents:
                    if agent.current_node:
                        agent_pos = self.graph.nodes[agent.current_node]['pos']
                        dist = math.dist(agent_pos, first_station_pos)
                        if dist < min_dist:
                            min_dist = dist
                            best_agent = agent
                            
                if not best_agent:
                    best_agent = idle_agents[0]
                    
                idle_agents.remove(best_agent)
                
                task['task_assigned'] = True
                task['agent_id'] = best_agent.agentId
                best_agent.agent_state = "EXECUTING"
                best_agent.current_task = task
                
            time.sleep(0.5)