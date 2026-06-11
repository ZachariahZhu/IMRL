import math
import uuid
import time
import threading
import heapq

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


        # while any(not t['task_assigned'] for t in self.task_management.task_list):
            
        #     #找车
        #     idle_agents = [a for a in self.agents.agents if a.agent_state == "IDLE"]
            
        #     if not idle_agents:
        #         time.sleep(0.5)
        #         continue
                
        
        #     agent = idle_agents[0]
                
        #     try:
        #         task = next(t for t in self.task_management.task_list if not t['task_assigned'])
        #     except StopIteration:
        #         break
                
        #     path_nodes, path_edges = self.build_path_for_task(task, agent.current_node)
        #     nodes = self.build_order_nodes(path_nodes, task)
        #     edges = self.build_order_edges(path_nodes, path_edges)
            
        #     task['task_assigned'] = True
        #     agent.agent_state = 'EXECUTING'
        #     agent.current_task = task
            
        #     agent.order_interface.generate_order_message(
        #         agent=agent,
        #         orderId=str(self.agents.order_header_id),
        #         order_updateId=0,
        #         nodes=nodes,
        #         edges=edges
        #     )
        #     time.sleep(0.5)


        for agent in self.agents.agents:
            while not agent.agvPosition:
                time.sleep(0.5)
            
            min_dist = float('inf')
            






    def build_path_for_task(self, task: dict, start_node: str) -> tuple:
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
        """
        combined_nodes= []
        combined_edges= []
        current= start_node
        #1.规划每一站的路径
        for station in task['stations']:
            target_node= station['nodeId']
            nodes,edges= self.path_planning.astar_search(current, target_node)

            if not combined_nodes:
                combined_nodes.extend(nodes)
            else:
                combined_nodes.extend(nodes[1:])#跳过第一个节点（它是上个路段的最后一个点），避免重复
            combined_edges.extend(edges)
            current= target_node

        #2.找到最近的休息点
        nearest_dwelling= None
        min_dist = float('inf')
        pos_current = self.graph.nodes[current]['pos']
        
        for d_node in self.graph.dwelling_nodes:
            pos_d=self.graph.nodes[d_node]['pos']
            dist= math.dist(pos_current, pos_d)
            if dist < min_dist:
                min_dist= dist
                nearest_dwelling= d_node
        #3.规划返回休息点的路径
        nodes, edges= self.path_planning.astar_search(current, nearest_dwelling)
        if combined_nodes:
            combined_nodes.extend(nodes[1:])#跳过第一个节点，避免重复
        else:
            combined_nodes.extend(nodes)
        combined_edges.extend(edges)

        return (combined_nodes, combined_edges)
    


        
    def build_order_nodes(self, path_nodes: list, task: dict) -> list:
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
              for the entry matching vehicleTypeId 'Longitudinal_Conveyor'.
              If the value is None or the string "None", use theta=None.

        Returns a list of node dicts:
            [{"nodeId": n, "x": x, "y": y, "theta": theta_or_None, "actions": [...]}, ...]

        Hint: build a lookup dict {station_nodeId: station} first, then iterate
        through path_nodes and check each node against the lookup.
        To detect the 'before TRANSFER' node, look ahead:
            if i + 1 < len(path_nodes) and path_nodes[i+1] is a TRANSFER station:
                add init_fine_positioning to path_nodes[i]
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
                        "blockingType": "HARD"#动作完成后才能继续下一个点
                    })
                elif st['actionType'] == 'process':#加工站点
                    actions.append({
                        "actionType": "process",
                        "actionId": str(uuid.uuid4()),
                        "blockingType": "HARD",
                        "processingTime": st['processingTime']#加工时长
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
            if node_props:
                for prop in node_props:
                    if prop['vehicleTypeId'] == 'Longitudinal_Conveyor':
                        val=prop.get('theta')
                        if val is not None and val != "None":
                            theta= float(val)
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

        Returns:
            [{
              "edgeId": path_edges[i],
              "startNodeId": path_nodes[i],
              "endNodeId": path_nodes[i+1],
              "actions": []}]
        """
        edges_result= []
        for i,e_id in enumerate(path_edges):
            edges_result.append({
                "edgeId": e_id,
                "startNodeId": path_nodes[i],
                "endNodeId": path_nodes[i+1],
                "actions": []
            })
        return edges_result
    


class PathPlanning:
    """
    A* shortest-path search over the graph.

    Task 6: Implement all three methods.
    Graph interfaces you will likely need:
        self.graph.get_connected_nodes(node_id)  -> list of neighbour node IDs
        self.graph.get_connected_edge(a, b)       -> edge ID connecting a and b
        self.graph.nodes[node_id]['pos']           -> (x, y) position tuple
    """

    def __init__(self, config_data, graph) -> None:
        self.config_data = config_data
        self.graph = graph

    def astar_search(self, start_node: str, goal_node: str) -> tuple:
        """
        Find the shortest path from start_node to goal_node using A*.

        Returns:
            (path_nodes, path_edges)
            path_nodes : ordered list of node IDs,  e.g. ["N5", "N1", "N7", "N2"]
            path_edges : ordered list of edge IDs,  e.g. ["E21", "E1", "E2"]
            len(path_edges) == len(path_nodes) - 1

        Returns (None, None) if no path exists.

        Task 6 hints:
            - Handle the edge case start_node == goal_node first:
                  return ([start_node], [])
            - Use a min-heap (heapq) ordered by f = g + h:
                  import heapq
                  open_set = []
                  heapq.heappush(open_set, (f, node_id))
            - g(n) = accumulated travel cost from start to n.
                  Increment it with self.get_distance(current, neighbour).
            - h(n) = self.get_h(n, goal_node)  -- Euclidean, never overestimates.
            - Keep a came_from dict to reconstruct the path on success.
            - Build path_edges by calling self.graph.get_connected_edge(a, b)
              for each consecutive pair in the reconstructed path_nodes.
        """
        if start_node == goal_node:
            return ([start_node], [])#1.起点即终点情况
        
        #2.初始化
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0.0}

        while open_set:
            current_f, current_node = heapq.heappop(open_set)#3.取出f值最小的节点
            #4.找到终点，并回溯
            if current_node == goal_node:
                path_nodes = [current_node]
                while current_node in came_from:
                    current_node = came_from[current_node]
                    path_nodes.append(current_node)
                    
                path_nodes.reverse()#翻转列表（因为是从终点倒推到起点的）
                #5.获取边
                path_edges = []
                for i in range(len(path_nodes) - 1):
                    edge = self.graph.get_connected_edge(path_nodes[i], path_nodes[i + 1])
                    path_edges.append(edge)
                return (path_nodes, path_edges)
            #6.遍历邻居节点
            for neighbour in self.graph.get_connected_nodes(current_node):
                tentative_g_score = g_score[current_node]+self.get_distance(current_node, neighbour)

                if neighbour not in g_score or tentative_g_score < g_score[neighbour]:
                    came_from[neighbour] = current_node
                    g_score[neighbour] = tentative_g_score
                    #f= g+h
                    f= tentative_g_score + self.get_h(neighbour, goal_node)
                    heapq.heappush(open_set, (f, neighbour))
        return (None, None)
                
                       
        

    def get_h(self, current_node: str, goal_node: str) -> float:
        """
        Heuristic -- Euclidean distance from current_node to goal_node.

        Task 6: Use self.graph.nodes[node_id]['pos'] for (x, y) coordinates,
        then return math.dist(pos_current, pos_goal).
        This heuristic is admissible (straight-line <= actual path length).
        """
        pos_current = self.graph.nodes[current_node]['pos']
        pos_goal = self.graph.nodes[goal_node]['pos']
        return math.dist(pos_current, pos_goal)#计算欧式距离（当前坐标到终点坐标）


    def get_distance(self, start_node: str, goal_node: str) -> float:
        """
        Actual edge cost -- Euclidean distance between two adjacent nodes.

        Task 6: Same formula as get_h(); used as the g-score increment per step.
        """
        pos_start = self.graph.nodes[start_node]['pos']
        pos_goal = self.graph.nodes[goal_node]['pos']
        return math.dist(pos_start, pos_goal)#计算欧式距离（起点坐标到终点坐标）