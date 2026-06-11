'''
This script is an example code to be used for individual task 2 and 3. 
The basic structure is provided, but the students need to implement the missing parts.
'''

import json
import os
import time
import argparse
from datetime import datetime
import paho.mqtt.client as mqtt
from jsonschema import validate, ValidationError
import math

_STATE_SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "interface", "state.schema"
)

def _load_state_schema():
    with open(os.path.normpath(_STATE_SCHEMA_PATH), "r", encoding="utf-8") as f:
        return json.load(f)

_STATE_SCHEMA = _load_state_schema()

class Robot:
    def __init__(self, name):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_node_id = None

        self.nodes = []         # Released nodes from order
        self.edges = []         # Released edges from order
        self.trajectory = []    # List of (x, y) waypoints
        self.current_index = 0  # Index of current waypoint

        self.status_update_needed = False  # Flag for event-triggered status update

        self.filtered_x = None
        self.filtered_y = None
        self.filtered_theta = None
        self.alpha = 0.3 #滤波系数，越小越平滑但响应越慢

        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.max_linear_accel = 1.0  # 加大线加速度
        self.max_angular_accel = 0.5  # 加大角加速度


    def update_pose(self, msg):
        # 解码并提取当前真实位置信息
        payload_str = msg.payload.decode('utf-8')
        pose_data = json.loads(payload_str)
        raw_x = pose_data['position']['x']
        raw_y = pose_data['position']['y']
        
        z = pose_data['orientation']['z']
        w = pose_data['orientation']['w']
        raw_theta = 2.0 * math.atan2(z, w)

        # 低通滤波（low-pass filter）
        if self.filtered_x is None:
            self.filtered_x = raw_x
            self.filtered_y = raw_y
            self.filtered_theta = raw_theta
        else:
            self.filtered_x = self.alpha * raw_x + (1 - self.alpha) * self.filtered_x
            self.filtered_y = self.alpha * raw_y + (1 - self.alpha) * self.filtered_y
            
            diff_theta = raw_theta - self.filtered_theta
            while diff_theta > math.pi: diff_theta -= 2 * math.pi
            while diff_theta < -math.pi: diff_theta += 2 * math.pi
            self.filtered_theta += self.alpha * diff_theta

        self.x = self.filtered_x
        self.y = self.filtered_y
        self.theta = self.filtered_theta#滤后坐标给机器人

            
    def receive_order(self, msg):
        payload_str = msg.payload.decode("utf-8")
        order_data = json.loads(payload_str)

        self.order_id = order_data.get('orderId', '')
        self.order_update_id = order_data.get('orderUpdateId', 0)

        self.trajectory = []  
        nodes = order_data.get("nodes", [])
        for node in nodes:
            if node.get('released') == True:
                x= node['nodePosition']['x']
                y= node['nodePosition']['y']
                node_id = node['nodeId']#额外提取节点ID
                self.trajectory.append((x,y,node_id)) # 坐标和节点ID一起存储
    
        self.current_index = 0     # Reset index
        self.last_node_id = None
        self.status_update_needed = True  # Order receipt triggers a status update

    def build_status_message(self):#当机器人到达一个节点，它需要发送一个符合VDA5050的反馈
        status = {
            "headerId": 0,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "version":"2.0.0",
            "manufacturer": "IMRL",
            "serialNumber": self.name,
            "orderId": getattr(self, 'order_id', "unknown_order"),
            "orderUpdateId": getattr(self, 'order_update_id', 0),
            #将刚刚的节点ID报告回去
            "lastNodeId": self.last_node_id if self.last_node_id else "",
            "lastNodeSequenceId": 0,
            "nodeStates": [],
            "edgeStates": [],
            "agvPosition": {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "positionInitialized": True,
                "mapId": "Map_1"
            },
            "batteryState": {
                "batteryCharge": 100,
                "charging": False
            },
            "operatingMode": "AUTOMATIC",
            "errors": [],
            "information": [],
            "driving": False,
            "actionStates": [],
            "safetyState": {"eStop":"NONE","fieldViolation":False}
        }
        return status


def follow_trajectory(robot: Robot):
    """
    Follow the trajectory by:
    - Computing control commands based on current pose and next waypoint
    - Publishing movement commands
    - Advancing to next waypoint when close enough

    Students should:
    - Implement trajectory following logic here (e.g., PD controller)
    - Calculate linear and angular velocities
    - Decide when to advance to the next waypoint
    - Set robot.status_update_needed = True if a waypoint/node is reached
    """
    if robot.current_index < len(robot.trajectory):
        #取出目标点的坐标和节点ID
        target_x, target_y, target_node_id = robot.trajectory[robot.current_index]

        #1.计算距离角度
        dx = target_x - robot.x
        dy = target_y - robot.y
        distance = math.hypot(dx, dy)#直线距离  
        target_angle = math.atan2(dy, dx)#目标对机器人的角度

        #2.计算角度误差
        angle_diff = target_angle - robot.theta
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        #3.速度控制
        if abs(angle_diff) > 0.1: #如果角度误差较大，优先转向
            target_linear_vel = 0.0
            target_angular_vel = 2.0 * angle_diff #z转弯速度与角度差成正比
        else: 
            #微调角度向前
            target_linear_vel = min(5.0 * distance, 10.0)  # 提高基础速度和上限
            target_angular_vel = 3.0 * angle_diff         # 提高转弯速度

        #4.加速度限制
        if target_linear_vel > robot.last_linear_vel + robot.max_linear_accel:
            linear_vel = robot.last_linear_vel + robot.max_linear_accel
        elif target_linear_vel < robot.last_linear_vel - robot.max_linear_accel:
            linear_vel = robot.last_linear_vel - robot.max_linear_accel
        else:
            linear_vel = target_linear_vel

        #角速度限制
        if target_angular_vel > robot.last_angular_vel + robot.max_angular_accel:
            angular_vel = robot.last_angular_vel + robot.max_angular_accel
        elif target_angular_vel < robot.last_angular_vel - robot.max_angular_accel:
            angular_vel = robot.last_angular_vel - robot.max_angular_accel
        else:
            angular_vel = target_angular_vel

        robot.last_linear_vel = linear_vel
        robot.last_angular_vel = angular_vel

        #判断是否抵达目标点
        if distance < 0.2:
            robot.current_index += 1 #前往下一个目标点
            robot.last_node_id = target_node_id #更新最后到达的节点ID
            robot.status_update_needed = True #到达节点触发状态更新 

        cmd = {
            "linear": {"x": linear_vel, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_vel}
        }
        return cmd
    else:
        return {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

def send_status_update(client, topic, robot: Robot):
    status = robot.build_status_message()
    try:
        validate(instance=status, schema=_STATE_SCHEMA)
    except ValidationError as e:
        print(f"[STATE VALIDATION ERROR] {e.message} (path: {list(e.path)})")
    client.publish(topic, json.dumps(status))

def main(robot_name):
    robot = Robot(robot_name)

    topic_cmd = f"KIT/IMRL/{robot_name}/cmd"
    topic_pose = f"KIT/IMRL/{robot_name}/pose"
    topic_order = f"KIT/IMRL/{robot_name}/order"
    topic_state = f"KIT/IMRL/{robot_name}/state"

    def on_connect(client, userdata, flags, rc): # Called when the client connects to the broker
        client.subscribe(topic_pose)
        client.subscribe(topic_order)

    def on_message(client, userdata, msg): # Called when a message is received
        if msg.topic == topic_pose:
            robot.update_pose(msg)
        elif msg.topic == topic_order:
            robot.receive_order(msg)

    client = mqtt.Client() # Create a new MQTT client instance
    client.on_connect = on_connect # Set the on_connect callback
    client.on_message = on_message # Set the on_message callback
    client.connect("localhost", 1883, 60)
    client.loop_start()

    last_status_time = time.time()

    while True:
        cmd = follow_trajectory(robot)
        client.publish(topic_cmd, json.dumps(cmd))

        # Send status update if an event occurred or periodically
        if robot.status_update_needed or time.time() - last_status_time >= 30:
            send_status_update(client, topic_state, robot)
            robot.status_update_needed = False
            last_status_time = time.time()

        time.sleep(0.1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default="mouse001") #Change to your robot name or parse it from command line
    args = parser.parse_args()
    main(args.robot)
