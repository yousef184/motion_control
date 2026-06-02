'''
This script is an example code to be used for individual task 2 and 3. 
The basic structure is provided, but the students need to implement the missing parts.
'''

import json
import math  #needed
import os
import time
import argparse
from datetime import datetime
import paho.mqtt.client as mqtt
from jsonschema import validate, ValidationError

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

    def update_pose(self, msg):
        # TODO: Parse pose update from payload 
        # 每次收到 pose 消息，更新 self.x/self.y/self.theta
        data = json.loads(msg.payload.decode("utf-8"))
        pos = data.get("position", {})
        self.x = pos.get("x", self.x)
        self.y = pos.get("y", self.y)
        #self.theta = pos.get("theta", self.theta)  position里没有Theta！！！
        ori = data.get("orientation", {})
        z = ori.get("z", 0.0)
        w = ori.get("w", 1.0)
        self.theta = 2.0 * math.atan2(z, w)

        # print(f"Pose updated: x={self.x}, y={self.y}, theta={self.theta}")
    

    def receive_order(self, msg):
        # TODO: Parse nodes and edges from the order message
        
        data = json.loads(msg.payload.decode("utf-8"))
        nodes = data.get("nodes", [])
        edges = data.get("edges", [])

        self.nodes = [n for n in nodes if n.get("released", False)]
        self.edges = [e for e in edges if e.get("released", False)]

        # TODO: Generate trajectory based on node positions
        self.trajectory = []       # Replace with generated waypoints
        for n in self.nodes:
            pos = n.get("nodePosition", {})
            x, y = pos.get("x"), pos.get("y")
            if x is not None and y is not None:
                self.trajectory.append((x, y))

        self.current_index = 0     # Reset index
        self.last_node_id = None
        self.status_update_needed = True  # Order receipt triggers a status update
        print("[ORDER] trajectory:", self.trajectory)

        #点 RESET 后，终端打印出轨迹坐标


    def build_status_message(self):
        # TODO: Construct and return VDA5050-compliant status message
        # An example. Add more fields as needed
        # status = {
        #     "timestamp": datetime.utcnow().isoformat() + "Z",
        #     "serialNumber": self.name,
        #     "agvPosition": {
        #         "x": self.x,
        #         "y": self.y,
        #         "theta": self.theta,
        #         "mapId": "Map_1"
        #     },
        # }
        status = {
            "headerId": int(time.time() * 10) % 1000000,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "version": "2.0.0",
            "manufacturer": "KIT",
            "serialNumber": self.name,
            "orderId": "",
            "orderUpdateId": 0,
            "lastNodeId": "" if self.last_node_id is None else str(self.last_node_id),
            "lastNodeSequenceId": 0,
            "nodeStates": [],
            "edgeStates": [],
            "driving": self.current_index < len(self.trajectory),
            "actionStates": [],
            "batteryState": {"batteryCharge": 100.0, "charging": False},
            "operatingMode": "AUTOMATIC",
            "errors": [],
            "safetyState": {"eStop": "NONE", "fieldViolation": False},
    }
        #验证：终端里 [STATE VALIDATION ERROR] 消失
        return status


def follow_trajectory(robot: Robot): #pd控制器  到一个waypoint切下个点
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
        #parameters for P controller  Proportion+Integration+Differentiation
        k_v = 0.5 #线速度  v = Kp_dist * distance 
        k_w = 1.2 #角速度  w = Kp_ang * heading_error
        max_linear = 0.35
        max_angular = 1.0
        arrival_threshold = 0.2  #调试一下看看效果 
        #P 跑通 如超调明显，加 D  如总有固定偏差，再谨慎加 I


        target = robot.trajectory[robot.current_index]
        target_x, target_y = target
        dx = target_x - robot.x
        dy = target_y - robot.y
        dist = math.hypot(dx, dy) 
        #计算当前点与目标点的距离，hypot函数返回欧几里得距离
        

        # TODO: Compute control commands (linear_vel, angular_vel)
        # 默认值（先保留）
        linear_vel = 0.0
        angular_vel = 0.0
        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_heading - robot.theta),
            math.cos(target_heading - robot.theta)
        )
        # 将 heading_error 规范化到 [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi

        # 再覆盖默认值 + PD 控制器


        if abs(heading_error) > 0.6:
            linear_vel = 0.0
        else:
            linear_vel = min(max_linear, k_v * dist)
        angular_vel = max(-max_angular, min(max_angular, k_w * heading_error))
        

        # TODO: Check if the robot is close enough to the target
        # If so:
        # - robot.current_index += 1
        # - robot.last_node_id = ...
        # - robot.status_update_needed = True


        if dist < arrival_threshold:
            if robot.current_index < len(robot.nodes):
                robot.last_node_id = robot.nodes[robot.current_index].get("nodeId", "")
            robot.current_index += 1
            robot.status_update_needed = True
            linear_vel = 0.0
            angular_vel = 0.0
        #检测位置点
        print(
            f"idx={robot.current_index}, dist={dist:.3f}, "
            f"target=({target_x},{target_y}), pose=({robot.x:.2f},{robot.y:.2f},{robot.theta:.2f}), "
            f"heading_err={heading_error:.2f}, lin={linear_vel:.2f}, ang={angular_vel:.2f}"
        )
    
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
