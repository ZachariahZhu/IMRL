import json
import os
import datetime
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher

_FLEET_MANAGEMENT_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")
)


class OrderInterface:
    def __init__(self, config_data: dict, logging: object, order_topic: str, agentId: str) -> None:
        self.config_data = config_data
        self.order_topic = order_topic
        self.logging = logging
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=order_topic,
                                            client_id=f'order_publisher_agent_{self.agentId}', logging=self.logging)

    def generate_order_message(self, agent: object, orderId: str, order_updateId: int,
                               nodes: list, edges: list, start_sequence_idx: int = 0) -> None:
        #1.组装nodes
        nodes_msg = []
        for i, node in enumerate(nodes):
            n = {
                "nodeId": node["nodeId"],
                "sequenceId": (start_sequence_idx + i) * 2,    # 节点必定是偶数 0, 2, 4...
                "released": node.get("released", True),
                "nodePosition": {
                    "x": node["x"],
                    "y": node["y"],
                    "mapId": "Map_1"
                },
                "actions": node.get("actions", [])
            }
            #如果theta不为None，则添加到nodePosition中
            if node.get("theta") is not None:
                n["nodePosition"]["theta"] = node["theta"]
            nodes_msg.append(n)
        #2.组装edges(odd)
        edges_msg = []
        for i, edge in enumerate(edges):
            e = {
                "edgeId": edge["edgeId"],
                "sequenceId": (start_sequence_idx + i) * 2 + 1,    # 边必定是奇数 1, 3, 5...
                "released": edge.get("released", True),
                "startNodeId": edge["startNodeId"],
                "endNodeId": edge["endNodeId"],
                "actions": edge.get("actions", [])
            }
            if "trajectory" in edge:
                e["trajectory"] = edge["trajectory"]
            edges_msg.append(e)
        #3.组装VDA5050字典
        order_msg = {
            "headerId": agent.order_header_id,
            "timestamp": datetime.datetime.now(datetime.UTC).isoformat().replace("+00:00", "Z"),
            "version": "2.0.0",
            "manufacturer": "IMRL",
            "serialNumber": self.agentId,
            "orderId": orderId,
            "orderUpdateId": order_updateId,
            "nodes": nodes_msg,
            "edges": edges_msg
        }
        #4.发布消息
        with open("orderMessages.txt", "a") as f:
            f.write(str(order_msg) + "\n")
        self.mqtt_publisher.publish(order_msg, qos=0)
        #5.更新headerId
        agent.order_header_id += 1
        
