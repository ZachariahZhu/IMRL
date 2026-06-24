import math
import uuid
import time
import threading
import heapq
import json
from fleet_management.traffic_controller import TrafficController

            
import time

class FleetManagement:
    """
    Manages the fleet: computes paths and sends VDA 5050 orders to agents.

    Task 7: Integrate A* into the full order pipeline (multi-stop tasks).
        Implement the helper methods build_path_for_task(), build_order_nodes(),
        and build_order_edges(), then call them from fleet_manager().

    Task 10: Automate all transportation tasks.
        Run fleet_manager() in a loop inside a daemon thread so tasks are
        picked up and executed one after another until all are done.
    """

    def __init__(self, config_data, graph, agents, task_management,
                 simulation_start_time) -> None:
        self.simulation_start_time = simulation_start_time
        self.config_data = config_data
        self.graph = graph
        self.agents = agents
        self.task_management = task_management
        self.path_planning = PathPlanning(config_data=self.config_data,
                                         graph=self.graph)
        self.traffic_controller = TrafficController(self)
        threading.Thread(target=self.fleet_manager, daemon=True).start()
        


    def fleet_manager(self) -> None:
        """
        Send a movement order to the first agent.

        Every transportation task follows this movement structure:
            dwelling (start) -> pick station -> process station(s) -> drop station -> dwelling

        Task 3 — manually fill the 'nodes' and 'edges' lists for the first
        transportation task (T1) and pass them to generate_order_message().

        Fill each entry following these rules:
            Node dict:  {"nodeId": str, "x": float, "y": float,
                         "theta": float | None, "actions": list}
            Edge dict:  {"edgeId": str, "startNodeId": str, "endNodeId": str,
                         "actions": list}

        Action rules (see orderMessage_Example.json for a reference):
          - Node immediately BEFORE a TRANSFER station (pick/drop):
                {"actionType": "init_fine_positioning",
                 "actionId": "<unique-uuid-string>", "blockingType": "HARD"}
          - TRANSFER station node (actionType pick or drop):
                {"actionType": "pick",  "actionId": "...", "blockingType": "HARD"}
             or {"actionType": "drop",  "actionId": "...", "blockingType": "HARD"}
          - PROCESS station node:
                {"actionType": "process", "actionId": "...", "blockingType": "HARD",
                 "processingTime": <float from task>}

        Node positions and theta values can be read from lif_file.json or
        self.graph.nodes[node_id] (if Task 4 is implemented).
        Use str(uuid.uuid4()) to generate unique actionId strings.

        """
                #              The full journey must cover:
        #              N5->N2(pick)->N3(process)->N4(drop)->N14
        # nodes = [{"nodeId":"N5", "x": 1.05, "y": 0.9, "theta": None, "actions": []},
        #          {"nodeId": "N1", "x": 2.46, "y": 0.79, "theta": -1.57, "actions": []},
        #          #N7(fine positioning) 
        #          {"nodeId": "N7", "x": 2.46, "y": 2.25, "theta": None, "actions": [{"actionType": "init_fine_positioning", "actionId": str(uuid.uuid4()), "blockingType": "HARD"}]},
        #          {"nodeId": "N2", "x": 2.46, "y": 3.7, "theta": 1.57, "actions": [{"actionType": "pick", "actionId": str(uuid.uuid4()), "blockingType": "HARD"}]},
        #          {"nodeId": "N7", "x": 2.46, "y": 2.25, "theta": None, "actions": []},
        #          {"nodeId": "N8", "x": 4.0, "y": 2.25, "theta": None, "actions": []},
        #          #N3(process)
        #          {"nodeId": "N3", "x": 5.45, "y": 2.25, "theta": None, "actions": [{"actionType": "process", "actionId": str(uuid.uuid4()), "blockingType": "HARD", "processingTime": 1.0}]},
        #          {"nodeId": "N13", "x": 5.55, "y": 3.5, "theta": None, "actions": []},
        #             #N4(drop，N12 fine positioning)
        #          {"nodeId": "N12", "x": 4.0, "y": 3.5, "theta": None, "actions": [{"actionType": "init_fine_positioning", "actionId": str(uuid.uuid4()), "blockingType": "HARD"}]},
        #          {"nodeId": "N4", "x": 5.55, "y": 4.75, "theta": 0.0, "actions": [{"actionType": "drop", "actionId": str(uuid.uuid4()), "blockingType": "HARD"}]},
        #          {"nodeId": "N12", "x": 4.0, "y": 3.5, "theta": None, "actions": []},
        #          {"nodeId": "N13", "x": 5.55, "y": 3.5, "theta": None, "actions": []},
        #          {"nodeId": "N14", "x": 6.8, "y": 3.5, "theta": None, "actions": []}  # RETURN to dwelling
        # ]   # List of node dicts with nodeId, x, y, theta, actions
        # edges = [
        #     {"edgeId": "E21", "startNodeId": "N5", "endNodeId": "N1", "actions": []},
        #     {"edgeId": "E1", "startNodeId": "N1", "endNodeId": "N7", "actions": []},
        #     {"edgeId": "E2", "startNodeId": "N7", "endNodeId": "N2", "actions": []},
        #     {"edgeId": "E2", "startNodeId": "N2", "endNodeId": "N7", "actions": []},
        #     {"edgeId": "E3", "startNodeId": "N7", "endNodeId": "N8", "actions": []},
        #     {"edgeId": "E5", "startNodeId": "N8", "endNodeId": "N3", "actions": []},
        #     {"edgeId": "E17", "startNodeId": "N3", "endNodeId": "N13", "actions": []},
        #     {"edgeId": "E12", "startNodeId": "N13", "endNodeId": "N12", "actions": []},
        #     {"edgeId": "E19", "startNodeId": "N12", "endNodeId": "N4", "actions": []},
        #     {"edgeId": "E19", "startNodeId": "N4", "endNodeId": "N12", "actions": []},
        #     {"edgeId": "E12", "startNodeId": "N12", "endNodeId": "N13", "actions": []},
        #     {"edgeId": "E16", "startNodeId": "N13", "endNodeId": "N14", "actions": []}
               # List of edge dicts with edgeId, startNodeId, endNodeId, actions
        """
        Task 7 — replace the manual lists with A*:
            1. Pick the first unassigned task from self.task_management.task_list.
            2. Call self.build_path_for_task(task, agent.current_node).
            3. Call self.build_order_nodes(path_nodes, task) and
               self.build_order_edges(path_nodes, path_edges).
            4. Pass the results to generate_order_message() below.
            5. Mark task['task_assigned'] = True and agent.agent_state = 'EXECUTING'.
        """
        # task= next(t for t in self.task_management.task_list if not t['task_assigned'])#1.找到第一个未分配的任务
        # agent= self.agents.agents[0]#2.找到一个空闲车
        # path_nodes,path_edges = self.build_path_for_task(task, agent.current_node)
        # nodes= self.build_order_nodes(path_nodes, task)
        # edges= self.build_order_edges(path_nodes, path_edges)#3.自动路径
        # task['task_assigned'] = True
        # agent.agent_state = 'EXECUTING'#4.锁定车和任务
        # agent.order_interface.generate_order_message(
        #     agent=agent,
        #     orderId=str(self.agents.order_header_id),
        #     order_updateId=0,
        #     nodes=nodes,
        #     edges=edges#发送订单
        # )


        for agent in self.agents.agents:
            while not agent.agvPosition:
                time.sleep(0.5)

            nearest_node = agent.current_node

            if nearest_node not in self.graph.dwelling_nodes:
                best_dwell = self._select_nearest_available_dwelling(
                    nearest_node,
                    excluding_agent=agent
                )

                path_nodes, path_edges= self.path_planning.astar_search(
                    nearest_node, best_dwell, agent.vehicle_type_id
                )

                nodes = self.build_order_nodes(path_nodes,{"stations": []}, agent.vehicle_type_id)
                edges = self.build_order_edges(path_nodes, path_edges)

                agent.agent_state = 'EXECUTING'
                
                for n in nodes:
                    n['released'] = False
                for e in edges:
                    e['released'] = False
                if nodes:
                    nodes[0]['released'] = True

                agent.full_nodes = nodes
                agent.full_edges = edges
                agent.released_index = 1
                agent.tracked_current_idx = 0
                agent.order_update_id = 0
                agent.current_order_id = str(self.agents.order_header_id)

                agent.order_interface.generate_order_message(
                    agent=agent,
                    orderId=agent.current_order_id,
                    order_updateId=agent.order_update_id,
                    nodes=agent.full_nodes,
                    edges=agent.full_edges
                )
                self.agents.order_header_id += 1

                while agent.agent_state != 'IDLE':
                    time.sleep(0.5)

        while any(not t['task_assigned'] for t in self.task_management.task_list):
            
            # Find all idle agents
            idle_agents = [a for a in self.agents.agents if a.agent_state == "IDLE"]
            
            if not idle_agents:
                time.sleep(0.5)
                continue
                
            # Pick the first idle agent
            agent = idle_agents[0]

            # Task 2e: get the vehicle type of the selected agent
            vehicle_type_id = agent.vehicle_type_id
                
            try:
                task = next(t for t in self.task_management.task_list if not t['task_assigned'])
            except StopIteration:
                break

            # Task 2e: pass vehicle_type_id to build_path_for_task()
            path_nodes, path_edges = self.build_path_for_task(
                task,
                agent.current_node,
                vehicle_type_id
            )

            if path_nodes is None or path_edges is None:
                print(
                    f"No valid path found for {agent.agentId} "
                    f"with vehicle type {vehicle_type_id}"
                )
                time.sleep(0.5)
                continue

            # Task 2e: pass vehicle_type_id to build_order_nodes()
            nodes = self.build_order_nodes(
                path_nodes,
                task,
                vehicle_type_id
            )

            edges = self.build_order_edges(path_nodes, path_edges)
            
            task['task_assigned'] = True
            agent.agent_state = 'EXECUTING'
            agent.current_task = task
            
            # Start of Dynamic Zone Control integration
            for n in nodes:
                n['released'] = False
            for e in edges:
                e['released'] = False
            if nodes:
                nodes[0]['released'] = True

            agent.full_nodes = nodes
            agent.full_edges = edges
            agent.released_index = 1
            agent.tracked_current_idx = 0
            agent.order_update_id = 0
            agent.current_order_id = str(self.agents.order_header_id)
            
            agent.order_interface.generate_order_message(
                agent=agent,
                orderId=agent.current_order_id,
                order_updateId=agent.order_update_id,
                nodes=agent.full_nodes,
                edges=agent.full_edges
            )
            self.agents.order_header_id += 1
            """
            if "mouse001" ==agent.agentId:
                with open("mouse001.json","w", encoding="utf-8") as fdf:
                
                    json.dump(nodes, fdf, indent=4, ensure_ascii=False)
            print("SUCESS0")
            if "cat001" == agent.agentId:
                with open("cat001.json","w", encoding="utf-8") as fdf2:
                
                    json.dump(nodes, fdf2, indent=4, ensure_ascii=False)
            print("SUCESS")
            other = None
            """
            """
            with open("out.txt", "a") as fd:
                print("0",file=fd)
            if "mouse001" ==agent.agentId:
                with open("cat001.json", "r", encoding="utf-8") as f:
                    other = json.load(f)
            with open("out.txt","a") as fd:
                print("1",file=fd)
            if "cat001" == agent.agentId:
                with open("mouse001.json", "r", encoding="utf-8") as f2:
                    other = json.load(f2)
            with open("out.txt","a") as fd:
                print("2",file=fd)
            if other != None:
                with open("out.txt","a") as fd:
                    for node in other:
                            print(node.get("nodeId"), file=fd)
                    print("-------",file=fd)
                    for node in nodes:
                        print(node.get("nodeId"), file=fd)
                    print("++++++", file=fd)
            
            """
            time.sleep(0.5)

    def build_path_for_task(self, task: dict, start_node: str,
                            vehicle_type_id: str) -> tuple:
        """
        Task 7: Chain multiple A* searches to cover all stations in a task.

        Every task follows: dwelling -> pick -> process(es) -> drop -> dwelling

        A task's stations list covers only the waypoints (not the dwelling legs):
            [{"nodeId": "N4", "actionType": "pick"},
             {"nodeId": "N3", "actionType": "process", "processingTime": 1.0},
             {"nodeId": "N2", "actionType": "drop"}]

        Build the full path by planning one A* leg per station plus a return leg:
            Leg 0: start_node  -> stations[0]['nodeId']   (travel to pick)
            Leg 1: stations[0] -> stations[1]['nodeId']   (pick to process)
            Leg 2: stations[1] -> stations[2]['nodeId']   (process to drop)
            Return: last station -> nearest dwelling node

        After each astar_search() call, extend the combined lists by:
            - path_nodes: skip the first node of each new leg (it duplicates
              the last node of the previous leg).
            - path_edges: concatenate as-is.

        For the return leg, choose the nearest dwelling node from
        self.graph.dwelling_nodes using self.graph.nodes[n]['pos'].

        Returns (path_nodes, path_edges).

        Task 2e:
            Pass vehicle_type_id to every astar_search() call.
        """
        combined_nodes= []
        combined_edges= []
        current= start_node

        #1.规划每一站的路径
        for station in task['stations']:
            target_node= station['nodeId']

            nodes, edges = self.path_planning.astar_search(
                current,
                target_node,
                vehicle_type_id
            )

            if nodes is None or edges is None:
                return None, None

            if not combined_nodes:
                combined_nodes.extend(nodes)
            else:
                combined_nodes.extend(nodes[1:])#跳过第一个节点，避免重复

            combined_edges.extend(edges)
            current= target_node

        nearest_dwelling = self._select_nearest_available_dwelling(
            current,
            excluding_agent=None
        )

        if nearest_dwelling is None:
            return None, None

        #3.规划返回休息点的路径
        nodes, edges = self.path_planning.astar_search(
            current,
            nearest_dwelling,
            vehicle_type_id
        )

        if nodes is None or edges is None:
            return None, None

        if combined_nodes:
            combined_nodes.extend(nodes[1:])#跳过第一个节点，避免重复
        else:
            combined_nodes.extend(nodes)

        combined_edges.extend(edges)

        return (combined_nodes, combined_edges)
    
    def _is_dwelling_occupied_by_other_agent(self, dwelling_node: str,
                                             excluding_agent=None) -> bool:
        """
        Return True if another executing agent's remaining path includes the
        given dwelling node.
        """
        for other in self.agents.agents:
            if other is excluding_agent:
                continue
            if other.agent_state != 'EXECUTING' or not hasattr(other, 'full_nodes'):
                continue

            current_idx = getattr(other, 'tracked_current_idx', 0)
            if current_idx >= len(other.full_nodes):
                current_idx = max(0, len(other.full_nodes) - 1)

            for node in other.full_nodes[current_idx:]:
                if node.get('nodeId') == dwelling_node:
                    return True

            if getattr(other, 'current_node', None) == dwelling_node:
                return True

        return False

    def _select_nearest_available_dwelling(self, current_node: str,
                                           excluding_agent=None) -> str | None:
        """
        Choose the nearest dwelling node that is not already in another agent's
        current or planned path.
        """
        if not self.graph.dwelling_nodes:
            return None
        if current_node == "init":
            current_node == ""
                
        candidate_dwelling = sorted(
            self.graph.dwelling_nodes,
            key=lambda d: math.dist(
                
                self.graph.nodes[current_node]['pos'],
                self.graph.nodes[d]['pos']
            )
        )

        for d_node in candidate_dwelling:
            if not self._is_dwelling_occupied_by_other_agent(
                d_node,
                excluding_agent=excluding_agent
            ):
                return d_node

        return candidate_dwelling[0]


        
    def build_order_nodes(self, path_nodes: list, task: dict,
                          vehicle_type_id: str) -> list:
        """
        Task 7: Assign VDA 5050 actions to each node in the combined path.

        Action rules (matching the dwelling -> pick -> process -> drop -> dwelling structure):
            - For stations with actionType 'pick' or 'drop' (TRANSFER):
                * The node immediately BEFORE the station in path_nodes
                  gets an 'init_fine_positioning' action.
                * The station node itself gets a 'pick' or 'drop' action.
            - For stations with actionType 'process' (PROCESS):
                * The station node gets a 'process' action with
                  processingTime from station['processingTime'].
            - All other nodes (intermediate + dwelling): empty actions list.

        Use uuid.uuid4() to generate unique actionId strings.

        For each node, also look up:
            - x, y from self.graph.nodes[n]['pos']
            - theta from self.graph.nodes[n].get('vehicleTypeNodeProperties', [])
              for the entry matching the selected agent's vehicle_type_id.
              If the value is None or the string "None", use theta=None.

        Returns a list of node dicts:
            [{"nodeId": n, "x": x, "y": y, "theta": theta_or_None, "actions": [...]}, ...]

        Hint: build a lookup dict {station_nodeId: station} first, then iterate
        through path_nodes and check each node against the lookup.
        To detect the 'before TRANSFER' node, look ahead:
            if i + 1 < len(path_nodes) and path_nodes[i+1] is a TRANSFER station:
                add init_fine_positioning to path_nodes[i]

        Task 2e:
            Use vehicle_type_id instead of hardcoded 'Longitudinal_Conveyor'.
        """
        nodes_result= []

        station_lookup= {s['nodeId']: s for s in task['stations']}

        for i, n_id in enumerate(path_nodes):
            actions= []

            if n_id in station_lookup:
                st = station_lookup[n_id]

                if st['actionType'] in ['pick', 'drop']:
                    actions.append({
                        "actionType": st['actionType'],
                        "actionId": str(uuid.uuid4()),
                        "blockingType": "HARD"
                    })

                elif st['actionType'] == 'process':
                    actions.append({
                        "actionType": "process",
                        "actionId": str(uuid.uuid4()),
                        "blockingType": "HARD",
                        "processingTime": st['processingTime']
                    })#如果是站点，添加相应的动作

            if i+1<len(path_nodes):
                next_node = path_nodes[i+1]

                if next_node in station_lookup:
                    next_st=station_lookup[next_node]

                    if next_st['actionType'] in ['pick', 'drop']:
                        actions.append({
                            "actionType": "init_fine_positioning",
                            "actionId": str(uuid.uuid4()),
                            "blockingType": "HARD"
                        })#如果下一个节点是转运站，当前节点需定位

            node_props=self.graph.nodes[n_id].get('vehicleTypeNodeProperties', [])
            theta= None

            for prop in node_props:
                if prop.get('vehicleTypeId') == vehicle_type_id:
                    val=prop.get('theta')

                    if val is not None and val != "None":
                        theta= float(val)

                    break

            nodes_result.append({
                "nodeId": n_id,
                "x": self.graph.nodes[n_id]['pos'][0],
                "y": self.graph.nodes[n_id]['pos'][1],
                "theta": theta,
                "actions": actions
            })

        return nodes_result
    

    def build_order_edges(self, path_nodes: list, path_edges: list) -> list:
        """
        Task 7: Build the edges input list for generate_order_message().
        """
        edges_result= []
        for i,e_id in enumerate(path_edges):
            edge_dict = {
                "edgeId": e_id,
                "startNodeId": path_nodes[i],
                "endNodeId": path_nodes[i+1],
                "actions": []
            }
            if self.graph.edges[e_id].get("trajectory"):
                edge_dict["trajectory"] = self.graph.edges[e_id]["trajectory"]
            edges_result.append(edge_dict)
        return edges_result
    


class PathPlanning:
    """
    A* shortest-path search over the graph.

    Task 6: Implement A*.
    Task 2d: Make A* vehicle-type-aware.
    """

    def __init__(self, config_data, graph) -> None:
        config_data = config_data
        self.graph = graph

    def astar_search(self, start_node: str, goal_node: str,
                     vehicle_type_id: str) -> tuple:
        """
        Find the shortest path from start_node to goal_node using A*.

        Task 2d update:
        The search now receives vehicle_type_id and only expands edges
        compatible with that vehicle type.

        Returns:
            (path_nodes, path_edges)

        Example:
            path_nodes = ["N7", "N8", "N2"]
            path_edges = ["E7", "E8"]

        Returns:
            (None, None) if no path exists.
        """

        # 1. If start and goal are same, no movement is needed.
        if start_node == goal_node:
            return ([start_node], [])

        # 2. Priority queue for A*
        open_set = []
        heapq.heappush(open_set, (0.0, start_node))

        # 3. Stores best previous node for path reconstruction
        came_from = {}

        # 4. Cost from start node to each node
        g_score = {
            node_id: float("inf")
            for node_id in self.graph.nodes
        }
        g_score[start_node] = 0.0

        # 5. Estimated total cost: g + h
        f_score = {
            node_id: float("inf")
            for node_id in self.graph.nodes
        }
        f_score[start_node] = self.get_h(start_node, goal_node)

        # 6. Closed/visited set
        visited = set()

        while open_set:
            current_f, current_node = heapq.heappop(open_set)

            if current_node in visited:
                continue

            visited.add(current_node)

            # 7. Goal reached: reconstruct path
            if current_node == goal_node:
                path_nodes = [current_node]

                while current_node in came_from:
                    current_node = came_from[current_node]
                    path_nodes.append(current_node)

                path_nodes.reverse()

                # 8. Convert node path into edge path
                path_edges = []

                for i in range(len(path_nodes) - 1):
                    edge = self.graph.get_connected_edge(
                        path_nodes[i],
                        path_nodes[i + 1],
                        vehicle_type_id
                    )

                    if edge is None:
                        return (None, None)

                    path_edges.append(edge)

                return (path_nodes, path_edges)

            # 9. Explore neighbours, but only vehicle-compatible neighbours
            for neighbour in self.graph.get_connected_nodes(
                current_node,
                vehicle_type_id
            ):
                penalty = 0.0
                if neighbour == 'N11':
                    # Unconditionally penalize N11 to force a passing loop.
                    # mouse001 MUST use N11 (no other path to N3), but cat001 will reroute via N10->N14.
                    penalty = 5.0
                    
                tentative_g_score = (
                    g_score[current_node]
                    + self.get_distance(current_node, neighbour)
                    + penalty
                )

                if tentative_g_score < g_score[neighbour]:
                    came_from[neighbour] = current_node
                    g_score[neighbour] = tentative_g_score

                    f_score[neighbour] = (
                        tentative_g_score
                        + self.get_h(neighbour, goal_node)
                    )

                    heapq.heappush(
                        open_set,
                        (f_score[neighbour], neighbour)
                    )

        return (None, None)

    def get_h(self, current_node: str, goal_node: str) -> float:
        """
        Heuristic: Euclidean distance from current_node to goal_node.
        """
        pos_current = self.graph.nodes[current_node]["pos"]
        pos_goal = self.graph.nodes[goal_node]["pos"]

        return math.dist(pos_current, pos_goal)

    def get_distance(self, start_node: str, goal_node: str) -> float:
        """
        Actual edge cost: Euclidean distance between two adjacent nodes.
        """
        pos_start = self.graph.nodes[start_node]["pos"]
        pos_goal = self.graph.nodes[goal_node]["pos"]

        return math.dist(pos_start, pos_goal)