'''
This script is an example code to be used for individual task 1. 
'''

import json
import paho.mqtt.client as mqtt

class Robot:
    def __init__(self, name):
        self.name = name
        self.nodes = []
        self.edges = []

    def receive_order(self, msg):
        """
        Process the received order message.

        Students should:
        - Decode the JSON payload
        - Extract 'nodes' and 'edges' from the message
        - Filter only the 'released' ones
        - Store relevant information in self.nodes and self.edges

        Example (to be implemented):
        # for node in nodes:
        #     print(f"Node ID: {node['nodeId']} at ({node['x']}, {node['y']})")
        """
        payload_str = msg.payload.decode("utf-8")
        order_data = json.loads(payload_str) #二进制解码成字符串，然后解析成python字典

        #保存路径的列表
        route = []

        #遍历节点
        nodes = order_data.get("nodes", [])#提取节点列表

        for node in nodes:
            if node.get('released')==True:#检查节点是否释放

                x= node['nodePosition']['x']#提取x坐标
                y= node['nodePosition']['y']#提取y坐标

                route.append((x,y))#将坐标添加到路径列表
                
        #打印
        print("Received order successfully")
        print(f"A total of {len(route)} released nodes were extracted.")
        for index, pos in enumerate(route):
            print(f"Node {index}: x=({pos[0]}, {pos[1]})")
        print("...\n")



def on_connect(client, userdata, flags, rc):
    client.subscribe(userdata["topic_order"])

def on_message(client, userdata, msg):
    robot: Robot = userdata["robot"]

    if msg.topic == userdata["topic_order"]:
        robot.receive_order(msg)

def main():
    robot_name = "mouse001"
    topic_order = f"KIT/IMRL/{robot_name}/order"

    robot = Robot(robot_name)

    client = mqtt.Client(userdata={
        "robot": robot,
        "topic_order": topic_order
    })

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
