import json
import os
import time
import argparse
from datetime import datetime
import paho.mqtt.client as mqtt
from jsonschema import validate, ValidationError
import math

_STATE_SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "motion_control", "data", "interface", "state.schema"
)

def _load_state_schema():
    try:
        with open(os.path.normpath(_STATE_SCHEMA_PATH), "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        print("[WARNING] Schema file not found. State validation will be bypassed.")
        return {}

_STATE_SCHEMA = _load_state_schema()

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class Robot:
    def __init__(self, name):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_node_id = None

        self.nodes = []                 
        self.edges = []                 
        self.trajectory = []            
        self.current_index = 0          
        self.last_time = time.time()    

        # --- Code 2 Improvements: Filtering and Velocity Tracking ---
        self.pose_initialized = False
        self.alpha = 0.55                 # 0 < alpha < 1. Adjusted for better smoothing/responsiveness
        self.cmd_linear_v = 0.0           # Actual commanded linear velocity
        self.cmd_angular_v = 0.0          # Actual commanded angular velocity
        self.status_update_needed = False 

        # --- Task 5: Fine Positioning States ---
        self.docking_mode = False
        self.undocking_mode = False
        self.fine_pose_x = 0.0
        self.fine_pose_y = 0.0
        self.fine_pose_theta = 0.0
        self.station_detected = 0
        
        self.target_dock_x = 0.0
        self.target_dock_y = 0.0
        self.target_dock_theta = 0.0

    def update_pose(self, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        raw_x = payload["position"]["x"]
        raw_y = payload["position"]["y"]
        zt = payload["orientation"]["z"]
        w = payload["orientation"]["w"]
        raw_theta = math.atan2(2 * w * zt, 1 - 2 * zt * zt)

        # Code 2: Instant Initialization to avoid filter lag from (0,0)
        if not self.pose_initialized:
            self.x = raw_x
            self.y = raw_y
            self.theta = raw_theta
            self.pose_initialized = True
        else:
            a = self.alpha
            self.x = a * raw_x + (1.0 - a) * self.x
            self.y = a * raw_y + (1.0 - a) * self.y
            
            sum_sin = a * math.sin(raw_theta) + (1.0 - a) * math.sin(self.theta)
            sum_cos = a * math.cos(raw_theta) + (1.0 - a) * math.cos(self.theta)
            self.theta = math.atan2(sum_sin, sum_cos) 

    def update_fine_pose(self, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        self.station_detected = payload.get("station_detected", 0)
        
        if self.station_detected > 0 and "pose" in payload:
            self.fine_pose_x = payload["pose"]["x"]
            self.fine_pose_y = payload["pose"]["y"]
            self.fine_pose_theta = payload["pose"]["theta"]

    def receive_order(self, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        nodes = payload["nodes"]
        self.trajectory = []
        for node in nodes:
            if node.get("released", False):
                self.trajectory.append(node)
        
        self.current_index = 0      
        self.last_node_id = None
        self.status_update_needed = True

        # Code 2: Reset filter and velocities on new order
        self.pose_initialized = False   
        self.cmd_linear_v = 0.0
        self.cmd_angular_v = 0.0
        print(f"[ORDER] Received {len(self.trajectory)} nodes.")

    def build_status_message(self):
        # Code 2: Dynamic Header ID
        status = {
            "headerId": int(time.time() * 10) % 1000000, 
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "version": "2.0.0",
            "manufacturer": "IFL",
            "serialNumber": self.name,
            "orderId": "1",
            "orderUpdateId": 1,
            "zoneSetId": "DEFAULT_ZONE",
            "lastNodeId": self.last_node_id if self.last_node_id else "",
            "lastNodeSequenceId": 1,
            "driving": self.current_index < len(self.trajectory) or self.docking_mode or self.undocking_mode,
            "nodeStates": [],
            "edgeStates": [],
            "actionStates": [],
            "agvPosition": {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "mapId": "Map_1",
                "positionInitialized": True
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

def extract_docking_action(node):
    if "actions" in node:
        for action in node["actions"]:
            if action.get("actionType") == "init_fine_positioning":
                params = {p["key"]: p["value"] for p in action.get("actionParameters", [])}
                return params
    return None

def follow_trajectory(robot: Robot, client, topic_fine_start, topic_fine_stop):
    current_time = time.time()
    dt = current_time - robot.last_time
    if dt <= 0.0: dt = 0.1 

    # Code 2: P Controller Parameters
    k_v = 0.45 
    k_w = 1.1 
    MAX_LIN_VEL, MAX_ANG_VEL = 0.32, 0.75
    MAX_LIN_ACC, MAX_ANG_ACC = 0.25, 0.8  
    arrival_threshold = 0.10  

    linear_vel, angular_vel = 0.0, 0.0

    if robot.current_index < len(robot.trajectory):
        target = robot.trajectory[robot.current_index]
        docking_params = extract_docking_action(target)

        # 1. Trigger Fine Positioning Service Start
        if docking_params and not robot.docking_mode:
            robot.docking_mode = True
            
            s_x = docking_params["init_fine_pos_x"]
            s_y = docking_params["init_fine_pos_y"]
            s_theta = docking_params["init_fine_pos_theta"]
            
            dx = robot.x - s_x
            dy = robot.y - s_y
            spr_x = dx * math.cos(s_theta) + dy * math.sin(s_theta)
            spr_y = -dx * math.sin(s_theta) + dy * math.cos(s_theta)
            spr_theta = normalize_angle(robot.theta - s_theta)

            robot.target_dock_x = docking_params["fine_pos_control_x"]
            robot.target_dock_y = docking_params["fine_pos_control_y"]
            robot.target_dock_theta = docking_params["fine_pos_control_theta"]

            start_msg = {
                "x": spr_x, "y": spr_y, "theta": spr_theta,
                "station_id": docking_params["init_fine_pos_name"]
            }
            client.publish(topic_fine_start, json.dumps(start_msg))
            print(f"[DOCKING] Initiated fine positioning for {docking_params['init_fine_pos_name']}")

        # 2. Control Logic Selection
        if robot.docking_mode:
            # --- TASK 5 DOCKING CONTROL ---
            MAX_LIN_VEL = 0.05  
            MAX_ANG_VEL = 0.05

            if robot.station_detected > 0:
                dy = robot.target_dock_y - robot.fine_pose_y
                dx = robot.target_dock_x - robot.fine_pose_x
                
                dist = math.hypot(dx, dy)
                target_angle = math.atan2(dy, dx)
                heading_error = normalize_angle(target_angle - robot.fine_pose_theta)
                
                if dist < 0.05:
                    heading_error = normalize_angle(robot.target_dock_theta - robot.fine_pose_theta)

                # Turn then drive logic for docking
                if abs(heading_error) > 0.15:
                    linear_vel = 0.0
                else:
                    linear_vel = k_v * dist * math.cos(heading_error)
                angular_vel = k_w * heading_error

                if dist < 0.015 and abs(heading_error) < 0.05:
                    print("[DOCKING] Target Reached. Transitioning to undock/next node.")
                    robot.docking_mode = False
                    robot.undocking_mode = True
                    robot.current_index += 1
                    robot.last_node_id = target["nodeId"]
                    robot.status_update_needed = True
            else:
                linear_vel = 0.01
                angular_vel = 0.0

        else:
            # --- CODE 2 NORMAL NAVIGATION CONTROL ---
            if robot.undocking_mode:
                MAX_LIN_VEL = 0.05 
                stop_msg = {"value": True}
                client.publish(topic_fine_stop, json.dumps(stop_msg))
                robot.undocking_mode = False
                print("[UNDOCKING] Fine localization stopped. Returning to normal mode.")

            target_x = target['nodePosition']['x']
            target_y = target['nodePosition']['y']
            dx = target_x - robot.x
            dy = target_y - robot.y

            dist = math.hypot(dx, dy) # Code 2 true distance
            target_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(target_heading - robot.theta)

            # Code 2: Turn-then-drive logic
            if abs(heading_error) > 0.35:
                linear_vel = 0.0
            else:
                linear_vel = min(MAX_LIN_VEL, k_v * dist * math.cos(heading_error))
            
            angular_vel = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, k_w * heading_error))

            # Check if normal node reached
            if dist < arrival_threshold:
                robot.current_index += 1
                robot.last_node_id = target["nodeId"]
                robot.status_update_needed = True
                
                # Stop instantly to prevent overshooting
                linear_vel = 0.0
                angular_vel = 0.0
                robot.cmd_linear_v = 0.0
                robot.cmd_angular_v = 0.0

        # --- CODE 2 ACCELERATION RAMPING ---
        # Clamp desired velocities
        desired_linear = max(-MAX_LIN_VEL, min(MAX_LIN_VEL, linear_vel))
        desired_angular = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, angular_vel))   
        
        # Calculate max allowed change
        d_lin = desired_linear - robot.cmd_linear_v
        d_ang = desired_angular - robot.cmd_angular_v
        
        # Apply constraints
        d_lin = max(-MAX_LIN_ACC * dt, min(MAX_LIN_ACC * dt, d_lin))
        d_ang = max(-MAX_ANG_ACC * dt, min(MAX_ANG_ACC * dt, d_ang))
        
        # Apply step to actual commands
        robot.cmd_linear_v += d_lin
        robot.cmd_angular_v += d_ang

        cmd = {
            "linear": {"x": robot.cmd_linear_v, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": robot.cmd_angular_v}
        }
        
    else:
        robot.cmd_linear_v = 0.0
        robot.cmd_angular_v = 0.0
        cmd = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    robot.last_time = current_time
    return cmd

def send_status_update(client, topic, robot: Robot):
    status = robot.build_status_message()
    if _STATE_SCHEMA:
        try:
            validate(instance=status, schema=_STATE_SCHEMA)
        except ValidationError as e:
            print(f"[STATE VALIDATION ERROR] {e.message} (path: {list(e.path)})")
    client.publish(topic, json.dumps(status))

def main(robot_name):
    robot = Robot(robot_name)
    ros_hostname = robot_name

    topic_cmd = f"KIT/IMRL/{ros_hostname}/cmd"
    topic_pose = f"KIT/IMRL/{ros_hostname}/pose"
    topic_order = f"KIT/IMRL/{ros_hostname}/order"
    topic_state = f"KIT/IMRL/{ros_hostname}/state"
    
    topic_fine_start = f"KIT/IMRL/{ros_hostname}/fine_loc/start"
    topic_fine_stop = f"KIT/IMRL/{ros_hostname}/fine_loc/stop"
    topic_fine_pose = f"KIT/IMRL/{ros_hostname}/fine_loc/pose"

    def on_connect(client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(topic_pose)
        client.subscribe(topic_order)
        client.subscribe(topic_fine_pose)

    def on_message(client, userdata, msg):
        if msg.topic == topic_pose:
            robot.update_pose(msg)
        elif msg.topic == topic_order:
            robot.receive_order(msg)
        elif msg.topic == topic_fine_pose:
            robot.update_fine_pose(msg)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_start()

    last_status_time = time.time()

    while True:
        cmd = follow_trajectory(robot, client, topic_fine_start, topic_fine_stop)
        client.publish(topic_cmd, json.dumps(cmd))

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