import json
from vda5050_interface.mqtt_clients.mqtt_subscriber import MQTTSubscriber
from vda5050_interface.interfaces.order_interface import OrderInterface
import math


class Agents:
    """
    Container for all digital-twin agent objects.
    Creates one Agent per entry in agentsInitialization_file.json.
    """

    def __init__(self, config_data, graph, agents_initialization_data, logging,
                 simulation_start_time) -> None:
        self.simulation_start_time = simulation_start_time
        self.logging = logging
        self.config_data = config_data
        self.graph = graph
        self.order_header_id = 1
        self.agents = self.get_agents(agents_initialization_data)

    def get_agents(self, agents_initialization_data) -> list:
        """
        Create the Agent objects from agentsInitialization_file.json.

        The mock below creates a single agent with minimal attributes.
        Extend the Agent constructor call to pass any additional attributes
        that your implementation requires (e.g. current node, velocity, current task).

        Hint: agents_initialization_data['agents'] is a list.
              Each entry contains 'agentId', 'stateTopic', 'orderTopic',
              'agentPosition' (x, y, theta), 'agentVelocity', etc.
        """
        
        # Task 5: Extend this to pass all attributes your Agent class needs.
        # You may also add any additional attributes to the Agent constructor as needed
        # in the folowing tasks.
        agents = [
            Agent(
                agents=self,
                agentId=entry['agentId'],
                vehicle_type_id=entry['vehicleTypeId'],
                agent_state='IDLE',
                agent_order_topic=entry['orderTopic'],
                agent_state_topic=entry['stateTopic'],
                logging=self.logging,
                x=entry['agentPosition']['x'],
                y=entry['agentPosition']['y'],
                theta=entry['agentPosition']['theta'],
                extGraph=self.graph
            )
            for entry in agents_initialization_data['agents']
        ]
        return agents


class Agent:
    """
    Digital twin of one simulated mobile robot.

    Attributes updated here must mirror the real robot's state as reported
    via VDA 5050 state messages (received in state_callback).
    """

    def __init__(self, agents, agentId, vehicle_type_id, agent_state_topic, agent_order_topic,
                 agent_state, logging,x,y,theta,extGraph) -> None:
        # ── Core references ───────────────────────────────────────────────────
        self.agents = agents          # parent Agents container
        self.agentId = agentId
        self.vehicle_type_id = vehicle_type_id

        # ── Communication ─────────────────────────────────────────────────────
        self.state_topic = agent_state_topic
        self.order_topic = agent_order_topic
        self.logging = logging
        self.mqtt_subscriber_state = MQTTSubscriber(
            config_data=self.agents.config_data, logging=self.logging,
            on_message=self.state_callback, channel=self.state_topic,
            client_id=f'state_subscriber_agent_{self.agentId}')
        self.order_interface = OrderInterface(
            config_data=self.agents.config_data, logging=logging,
            order_topic=self.order_topic, agentId=self.agentId)
        self.order_header_id = 1
        self.order_update_id = 1

        # ── State (updated by state_callback) ────────────────────────────────
        self.agent_state = agent_state   # 'IDLE' | 'EXECUTING'
        self.agvPosition = {
            "x": x,
            "y": y,
            "theta": theta
        }            # last known position from state message
        self.safetyState = {}

        # ── Task & path ───────────────────────────────────────────────────────
        self.loaded = False              # True while carrying a load
        self.nodesInitialized= False
        self.current_node ="N5" # ?
        tempNodes = extGraph.nodes
        shortest = None
        position = None
        #with open("debug.txt","w") as ff:
        #print("TEMPNODES",(x,y),file=ff)
        #print(tempNodes,file=ff)
        for tempNode in tempNodes.values():
            #print(tempNode.get("pos"),tempNode.get("nodeId"),file=ff)
            dist = math.dist(tempNode.get("pos"),(x,y))
            if shortest == None:                    
                shortest = dist
                position = tempNode.get("nodeId")
            elif dist < shortest:
                shortest = dist
                position = tempNode.get("nodeId")
        #print(position,file=ff)
        #print(shortest,file=ff)


            #print("TEMPNODE")
            #print(tempNode)
        #    dist = math.dist((tempNode.get("nodePosition").get("x"),tempNode.get("nodePosition").get("y")),(x,y))
        #    if shortest == None:
        #        shortest = dist
         #       position = tempNode.get("nodeId")
        #    elif dist < shortest:
        #        shortest = dist
         #       position = tempNode.get("nodeId")
            
        self.current_node= position
        self.current_task = None
        self.current_path_nodes = []
    def state_callback(self, client, userdata, msg) -> None:
        """
        Called automatically whenever the simulation publishes a state message.

        Parse the incoming state message and update the agent's attributes.

        The message payload is a JSON-encoded VDA 5050 state message.
        See data/input_files/stateMessage_Example.json for the full structure.

        Required steps:
          1. Decode and parse the JSON payload.
          2. Update self.agvPosition from state_msg['agvPosition'].
          3. Update self.current_node from state_msg['lastNodeId']  (Task 5 attribute).
          4. Detect task completion:
               - When state_msg['nodeStates'] AND state_msg['edgeStates'] are empty
                 AND all actions in state_msg['actionStates'] have 'actionStatus' == 'FINISHED',
                 the robot has finished its current order.
               - Set self.current_task['task_completed'] = True.
                 (self.current_task holds a reference to the task dict inside
                 task_management.task_list, so updating it here updates the list directly.)
               - Set self.agent_state = 'IDLE'.

        Hint: use json.loads(msg.payload.decode()) to parse the message.
        """
        self.logging.info(
            f"Client {self.mqtt_subscriber_state.client_id} received message "
            f"`{msg.payload.decode()}` from topic `{msg.topic}`.")

        state_msg = json.loads(msg.payload.decode('utf-8'))#解码
        
        if 'agvPosition' in state_msg:
            self.agvPosition = state_msg['agvPosition']
        if not self.nodesInitialized:
            self.agvPosition = state_msg['agvPosition']
            self.nodesInitialized=True
        if 'lastNodeId' in state_msg and state_msg['lastNodeId']:
            self.current_node = state_msg['lastNodeId']
            
        self.actionStates = state_msg.get('actionStates', [])
        self.driving = state_msg.get('driving', False)

        # Task 4 Collision Avoidance: Track the remaining path nodes
        self.current_path_nodes = [n['nodeId'] for n in state_msg.get('nodeStates', [])]
        if self.current_node:
            self.current_path_nodes.append(self.current_node)

        nodes_empty = len(state_msg.get('nodeStates', [])) <= 1
        edges_empty = len(state_msg.get('edgeStates', [])) == 0 #判断是否完成
        actions_finished = True
        for action in state_msg.get('actionStates', []):
            if action.get('actionStatus') != 'FINISHED':
                actions_finished = False
                break
        if self.agent_state == 'EXECUTING' and nodes_empty and edges_empty and actions_finished:
            if self.current_task is not None:
                self.current_task['task_completed'] = True
            self.agent_state = 'IDLE'
            