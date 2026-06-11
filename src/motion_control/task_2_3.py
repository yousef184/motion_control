'''
Integrated VDA5050 Robot Controller
Combines PID control (with anti-windup) with robust pose initialization and angle normalization.
'''

import json
import math
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

        self.nodes = []                 # Released nodes from order
        self.edges = []                 # Released edges from order
        self.trajectory = []            # List of (x, y) waypoints
        self.current_index = 0          # Index of current waypoint
        
        # PID Controller tracking variables
        self.prev_distance_error = 0.0  
        self.prev_angle_error = 0.0
        self.sum_distance_error = 0.0   # Added Integral term for distance
        self.sum_angle_error = 0.0      # Added Integral term for angle
        self.last_time = time.time()   

        # Filter and State variables
        self.pose_initialized = False
        self.x_f = 0.0
        self.y_f = 0.0
        self.theta_f = 0.0
        self.alpha = 0.55               # 0 < alpha < 1. Smaller number = smoother
        
        self.status_update_needed = False 
        self.cmd_linear_v = 0.0
        self.cmd_angular_v = 0.0

    def update_pose(self, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        
        # Safe extraction using get()
        pos = payload.get("position", {})
        raw_x = pos.get("x", self.x_f)
        raw_y = pos.get("y", self.y_f)
        
        ori = payload.get("orientation", {})
        z = ori.get("z", 0.0)
        w = ori.get("w", 1.0)
        raw_theta = 2.0 * math.atan2(z, w)

        # Snap to first position, otherwise apply low-pass filter
        if not self.pose_initialized:
            self.x_f = raw_x
            self.y_f = raw_y
            self.theta_f = raw_theta
            self.pose_initialized = True
        else:
            a = self.alpha
            self.x_f = a * raw_x + (1.0 - a) * self.x_f
            self.y_f = a * raw_y + (1.0 - a) * self.y_f
            
            sum_sin = a * math.sin(raw_theta) + (1.0 - a) * math.sin(self.theta_f)
            sum_cos = a * math.cos(raw_theta) + (1.0 - a) * math.cos(self.theta_f)
            self.theta_f = math.atan2(sum_sin, sum_cos) 
        
        self.x = self.x_f
        self.y = self.y_f
        self.theta = self.theta_f

    def receive_order(self, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        nodes = payload.get("nodes", [])
        edges = payload.get("edges", [])

        self.nodes = [n for n in nodes if n.get("released", False)]
        self.edges = [e for e in edges if e.get("released", False)]

        # Generate trajectory based on node positions
        self.trajectory = []
        for n in self.nodes:
            pos = n.get("nodePosition", {})
            x, y = pos.get("x"), pos.get("y")
            if x is not None and y is not None:
                self.trajectory.append({"nodeId": n.get("nodeId"), "x": x, "y": y})

        self.current_index = 0
        self.last_node_id = None
        self.status_update_needed = True
        
        # Reset velocities, poses, and PID memory for new order
        self.pose_initialized = False
        self.cmd_linear_v = 0.0
        self.cmd_angular_v = 0.0
        self.prev_distance_error = 0.0
        self.prev_angle_error = 0.0
        self.sum_distance_error = 0.0
        self.sum_angle_error = 0.0
        
        print("[ORDER] trajectory:", [(t["x"], t["y"]) for t in self.trajectory])

    def build_status_message(self):
        status = {
            "headerId": int(time.time() * 10) % 1000000,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "version": "2.0.0",
            "manufacturer": "IFL/KIT",
            "serialNumber": self.name,
            "orderId": "1",
            "orderUpdateId": 1,
            "zoneSetId": "DEFAULT_ZONE",
            "lastNodeId": str(self.last_node_id) if self.last_node_id else "",
            "lastNodeSequenceId": 1,
            "driving": self.current_index < len(self.trajectory),
            "nodeStates": [],
            "edgeStates": [],
            "actionStates": [],
            "agvPosition": {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "mapId": "Map_1",
                "positionInitialized": self.pose_initialized
            },
            "velocity": {
                "vx": self.cmd_linear_v,
                "vy": 0.0,
                "omega": self.cmd_angular_v
            },
            "batteryState": {
                "batteryCharge": 100.0,
                "charging": False
            },
            "safetyState": {
                "eStop": "NONE",
                "fieldViolation": False
            },
            "operatingMode": "AUTOMATIC",
            "errors": [],
            "information": [],
        }
        return status

def ramp_velocity(desired, current, max_acc, dt):
    """ Helper function to enforce maximum acceleration constraints. """
    max_delta = max_acc * dt
    delta = desired - current

    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta

    return current + delta

def follow_trajectory(robot: Robot):
    if robot.current_index < len(robot.trajectory):
        target = robot.trajectory[robot.current_index]
        target_x = target['x']
        target_y = target['y']

        # PID parameters (Proportional, Integral, Derivative)
        kp1, ki1, kd1 = 0.5, 0.05, 0.05   # Linear
        kp2, ki2, kd2 = 1.1, 0.10, 0.1   # Angular

        # Anti-windup clamping limits for integrals
        MAX_I_LIN = 2.0
        MAX_I_ANG = 2.0

        current_time = time.time()
        dt = current_time - robot.last_time
        if dt <= 0.0:
            dt = 0.001 

        dx = target_x - robot.x
        dy = target_y - robot.y
        
        # Corrected distance calculation (Euclidean)
        distance_error = math.hypot(dx, dy)
        
        # Calculate target heading and normalized error
        target_heading = math.atan2(dy, dx)
        angle_error = target_heading - robot.theta
        
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # Update Integral terms with Anti-Windup Clamping
        # (Only accumulate distance integral if we aren't heavily turning in place)
        if abs(angle_error) <= 0.35:
            robot.sum_distance_error += distance_error * dt
            robot.sum_distance_error = max(-MAX_I_LIN, min(MAX_I_LIN, robot.sum_distance_error))

        robot.sum_angle_error += angle_error * dt
        robot.sum_angle_error = max(-MAX_I_ANG, min(MAX_I_ANG, robot.sum_angle_error))

        # Derivative terms
        dv = (distance_error - robot.prev_distance_error) / dt
        da = (angle_error - robot.prev_angle_error) / dt

        # PID Control Output
        linear_vel = kp1 * distance_error + ki1 * robot.sum_distance_error + kd1 * dv
        angular_vel = kp2 * angle_error + ki2 * robot.sum_angle_error + kd2 * da

        # Turn-First Logic: Limit linear speed if facing the wrong way
        if abs(angle_error) > 0.35:
            linear_vel = 0.0

        # Constraints
        MAX_LIN_ACC, MAX_ANG_ACC = 0.25, 0.8
        MAX_LIN_VEL, MAX_ANG_VEL = 0.32, 0.75
        
        desired_linear = max(-MAX_LIN_VEL, min(MAX_LIN_VEL, linear_vel))
        desired_angular = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, angular_vel))   
        
        # Apply Acceleration Ramp
        linear_vel = ramp_velocity(desired_linear, robot.cmd_linear_v, MAX_LIN_ACC, dt)
        angular_vel = ramp_velocity(desired_angular, robot.cmd_angular_v, MAX_ANG_ACC, dt)

        # Update saved states
        robot.cmd_linear_v = linear_vel
        robot.cmd_angular_v = angular_vel
        robot.prev_distance_error = distance_error
        robot.prev_angle_error = angle_error
        robot.last_time = current_time

        # Arrival Check
        ARRIVAL_THRESHOLD = 0.10
        if distance_error < ARRIVAL_THRESHOLD:
            robot.current_index += 1
            robot.last_node_id = target.get("nodeId", "")
            robot.status_update_needed = True
            
            # Reset errors, integrals, and velocity for clean cornering
            robot.prev_distance_error = 0.0
            robot.prev_angle_error = 0.0
            robot.sum_distance_error = 0.0
            robot.sum_angle_error = 0.0
            robot.cmd_linear_v = 0.0
            robot.cmd_angular_v = 0.0

        return {
            "linear": {"x": linear_vel, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_vel}
        }
    else:
        # Stop completely when trajectory is done
        robot.cmd_linear_v = 0.0
        robot.cmd_angular_v = 0.0
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

    def on_connect(client, userdata, flags, rc): 
        client.subscribe(topic_pose)
        client.subscribe(topic_order)

    def on_message(client, userdata, msg):
        if msg.topic == topic_pose:
            robot.update_pose(msg)
        elif msg.topic == topic_order:
            robot.receive_order(msg)

    client = mqtt.Client()
    client.on_connect = on_connect 
    client.on_message = on_message 
    client.connect("172.22.222.238", 1883, 60)
    client.loop_start()

    last_status_time = time.time()

    while True:
        cmd = follow_trajectory(robot)
        client.publish(topic_cmd, json.dumps(cmd))

        # Send status update if an event occurred or periodically (every 30s)
        if robot.status_update_needed or time.time() - last_status_time >= 30:
            send_status_update(client, topic_state, robot)
            robot.status_update_needed = False
            last_status_time = time.time()

        time.sleep(0.1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default="mouse001") 
    args = parser.parse_args()
    main(args.robot)