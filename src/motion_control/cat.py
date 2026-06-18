'''
Integrated VDA5050 Robot Controller
Combines PD differential control with robust pose initialization and angle normalization.
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
        
        # PD Controller tracking variables
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

        self.fine_loc_trigger = None
        self.docking_active = False
        self.undocking_active = False
        self.state = None
        self.fine_pose = None
        self.station_detected = 0
        self.docking_err = 1000
        self.min_err = 10000
        
        self.con_speed = 0.1

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
        payload = json.loads(msg.payload.decode("utf-8"))
        nodes = payload.get("nodes", [])
        edges = payload.get("edges", [])
    
        self.nodes = [n for n in nodes if n.get("released", False)]
        self.edges = [e for e in edges if e.get("released", False)]
    
        self.trajectory = []
        i=-1
        for n in self.nodes:
            i+=1
            pos = n.get("nodePosition", {})
            x, y = pos.get("x"), pos.get("y")
    
            if x is None or y is None:
                continue
            
            # Default waypoint
            waypoint = {
                "nodeId": n.get("nodeId"),
                "x": x,
                "y": y,
                "is_docking_node": False
            }

            
            docking_action = None
            for action in n.get("actions", []):
                docking_action = action.get("actionType")
            waypoint["action"] = docking_action
            # ---- Normal waypoint ----
            if docking_action in ("pick","drop"):
                self.trajectory[i]["action"] = docking_action
                continue
            elif docking_action is None:
                self.trajectory.append(waypoint)
                continue
            self.trajectory.append(waypoint)                
            # ---- Docking waypoint handling ----
            params = {p["key"]: p["value"] for p in n.get("actions")[0].get("actionParameters", [])}
    
            station_x = float(params.get("init_fine_pos_x", 0.0))
            station_y = float(params.get("init_fine_pos_y", 0.0))
            station_theta = float(params.get("init_fine_pos_theta", 0.0))
    
            # --- Pre-docking waypoint (0.5 m away, straight line) ---
            pre_dock_distance = 0.4
            theta = float(params.get("fine_pos_control_theta", 0.0))
            pre_x = station_x- pre_dock_distance * math.cos(theta) + float(params.get("fine_pos_control_x", 0.0))
            pre_y = station_y- pre_dock_distance * math.sin(theta) + float(params.get("fine_pos_control_y", 0.0))
    
            pre_dock_wp = {
                "nodeId": f"{n.get('nodeId')}_pre_dock",
                "x": pre_x,
                "y": pre_y,
                "is_docking_node": True,
                "station_id": params.get("init_fine_pos_name"),
                "station_x": station_x,
                "station_y": station_y,
                "station_theta": station_theta,
                "localx" :float(params.get("fine_pos_control_x", 0.0)),
                "localy" : float(params.get("fine_pos_control_y", 0.0)),
                "localt" : float(params.get("fine_pos_control_theta", 0.0)),
            }
    
            self.trajectory.append(pre_dock_wp)
            
            
        
        self.current_index = 0
        self.last_node_id = None
        self.status_update_needed = True
    
        print("[ORDER] trajectory:")
        for wp in self.trajectory:
            print(wp)

    def robot_fine(self,msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        self.station_detected = payload.get("station_detected", 0)             
        if self.station_detected > 0:
            self.fine_pose = payload.get("pose")    

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

def calculate_relative_pose(robot_x, robot_y, robot_theta, station_x, station_y, station_theta):
    """
    Calculates the robot's relative pose (SPR) in the station's coordinate frame.
    """
    dx = robot_x - station_x
    dy = robot_y - station_y
    
    # Rotate into the station's coordinate frame
    x_rel = dx * math.cos(station_theta) + dy * math.sin(station_theta)
    y_rel = -dx * math.sin(station_theta) + dy * math.cos(station_theta)
    
    # Calculate relative angle and normalize to [-pi, pi]
    theta_rel = robot_theta - station_theta
    while theta_rel > math.pi: 
        theta_rel -= 2.0 * math.pi
    while theta_rel < -math.pi: 
        theta_rel += 2.0 * math.pi
        
    return x_rel, y_rel, theta_rel
    

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
        ARRIVAL_THRESHOLD= 0.1
        if distance_error < ARRIVAL_THRESHOLD:
            if target.get("is_docking_node")== True:
                # 2. Get Station Pose (MPS) from the target
                mps_x = target.get("station_x", 0.0)
                mps_y = target.get("station_y", 0.0)
                mps_theta = target.get("station_theta", 0.0)
                
                # 3. Calculate Relative Pose (SPR)
                spr_x, spr_y, spr_theta = calculate_relative_pose(
                    robot.x, robot.y, robot.theta, mps_x, mps_y, mps_theta
                )
                
                # 4. Construct the payload for KIT/IMRL/{ROS_HOSTNAME}/fine_loc/start
                robot.fine_loc_trigger = {
                    "x": spr_x,
                    "y": spr_y,
                    "theta": spr_theta,
                    "station_id": target.get("station_id", "")
                }

            
            robot.current_index += 1
            robot.last_node_id = target.get("nodeId", "")
            robot.status_update_needed = True
            
            
            # Reset errors for the next segment
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

def docking(robot: Robot):
    target = robot.trajectory[robot.current_index - 1]
    # 1. Failsafe: Stop if we lose sight of the station marker
    target = robot.trajectory[robot.current_index - 1]
    t_x = target.get("localx", 0.0)
    t_y = target.get("localy", 0.0)
    t_theta = target.get("localt", 0.0)
    flag = 0
    # 2. Extract Current Pose (Camera vs. Global Fallback)
    if not getattr(robot, 'fine_pose', None) or robot.station_detected == 0:
        # FALLBACK: Marker lost! Use global localization and transform into station frame
        s_x = target.get("station_x", 0.0)
        s_y = target.get("station_y", 0.0)
        s_theta = target.get("station_theta", 0.0)
        
        c_x, c_y, c_theta = calculate_relative_pose(
            robot.x, robot.y, robot.theta, 
            s_x, s_y, s_theta
        )
        print("global")
        # Optional: Print a warning so you know it's flying blind/using fallback
        # print("[DOCKING] Marker lost! Falling back to global pose estimation.")
    else:
        # PRIMARY: Marker visible! Use high-precision camera pose
        c_x = robot.fine_pose["x"]
        c_y = robot.fine_pose["y"]
        c_theta = robot.fine_pose["theta"]
        print("local")
        flag = 1

    # 3. Calculate Errors
    dx = t_x - c_x
    dy = t_y - c_y
    dist_error = math.hypot(dx, dy)
    
    # Helper to keep angles between -pi and pi
    def norm_angle(a):
        while a > math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    # Only calculating final angle error now
    final_angle_err = norm_angle(t_theta - c_theta)

    min_err = 10000
    # 4. Strict Safety Constraints
    MAX_LIN = 0.05  # 5 cm/s
    MAX_ANG = 0.15  # ~8.5 deg/s
    TOL_DIST = 0.08# 1.5 cm tolerance
    TOL_ANG = 0.03  # ~1.7 deg tolerance
    
    lin_vel, ang_vel = 0.0, 0.0
    

    # Ensure state tracker exists
    if not hasattr(robot, 'docking_step'):
        robot.docking_step = "TURN_1"

    # --- THE STATE MACHINE ---

    if robot.docking_step == "TURN_1":
        # STEP 1: Spin in place to match the final target orientation
        if abs(final_angle_err) > 0.03:
            ang_vel = max(-MAX_ANG, min(MAX_ANG, final_angle_err * 0.8))
            print(f"[DOCKING] TURN 1: Aligning to final angle. Angle error: {final_angle_err:.3f}")
        else:
            robot.docking_step = "MOVE"

    elif robot.docking_step == "MOVE":
        # STEP 2: Drive while maintaining that final orientation
        if abs(final_angle_err) > 0.03:
            robot.docking_step = "TURN_1"
        elif dist_error>0.05:
            lin_vel = max(-MAX_LIN, min(MAX_LIN, dist_error * 1))
            # Micro-adjustments to keep it locked onto the final angle while moving
            ang_vel = max(-MAX_ANG, min(MAX_ANG, final_angle_err * 0.8))
            print(f"[DOCKING] MOVE: Driving. Distance error: {dist_error:.3f}")
        else:
            print(dist_error,robot.min_err)
            print("\n✅ [DOCKING] SUCCESSFULLY DOCKED WITHIN TOLERANCES!\n")
            robot.docking_step = "TURN_1" 
            robot.docking_active = False  
            robot.status_update_needed = True 
            robot.undocking_active = True
            robot.state = "docking_complete"

            
    flag = 0
    if dist_error < robot.min_err:
        robot.min_err = dist_error
    return {
        "linear": {"x": lin_vel, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": ang_vel}
    }

def undocking(robot,topic,client):
    # Initialize undocking state once
    if not hasattr(robot, "undocking_active"):
        robot.undocking_active = True

    # Parameters
    MAX_LIN = 0.05
    UNDOCK_DIST = 0.06

    # Target (station pose)
    target = robot.trajectory[robot.current_index - 1]
    t_x = target.get("x",0.0)
    t_y = target.get("y",0.0)
    dx = t_x - robot.x
    dy = t_y - robot.y
    dist_error = math.hypot(dx, dy)

   
    dist_error = math.hypot(dx, dy)

    # Exit condition
    if dist_error <= UNDOCK_DIST:
        print("\n🔓 [UNDOCKING] COMPLETE\n")
        robot.undocking_active = False
        client.publish(topic, json.dumps({"value":True}))
        print("published to topic")
        return {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
        

    # Drive backward
    lin_vel = -min(MAX_LIN, (dist_error) * 0.8)

    print(f"[UNDOCKING] Backing up. Distance: {dist_error:.3f}")


    print(lin_vel)
    return {
        "linear": {"x": lin_vel, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
    }

def con_move(client,topicR,topicS,speed):
    client.publish(topicR, json.dumps({"value":0.2}))
    client.publish(topicS, json.dumps({"value":0.2}))
    print("conveying")
    time.sleep(7)
    client.publish(topicR, json.dumps({"value":0.0}))
    client.publish(topicS, json.dumps({"value":0.0}))
    pass


def convey(robot,client,topic,speed):
    station_name = robot.trajectory[robot.current_index-1]["station_id"]
    action_type = robot.trajectory[robot.current_index-1]["action"]
    
    station_topic = f"KIT/IMRL/{station_name[0:-2]}/conveyor/speed"
    print(station_topic)
    if ("_A" in station_name and action_type == "drop") or ("_B" in station_name and action_type == "pick"):
        con_move(client,topic,station_topic,speed)
    else:
        con_move(client,topic,station_topic,-speed)

    

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
    topic_loc = f"KIT/IMRL/{robot_name}/fine_loc/start"
    topic_loc_pose = f"KIT/IMRL/{robot_name}/fine_loc/pose"
    topic_stop_fine = f"KIT/IMRL/{robot_name}/fine_loc/stop"
    topic_conveyer_robot = f"KIT/IMRL/{robot_name}/conveyor/speed"

    def on_connect(client, userdata, flags, rc): 
        client.subscribe(topic_pose)
        client.subscribe(topic_order)
        client.subscribe(topic_loc_pose)

    def on_message(client, userdata, msg):
        if msg.topic == topic_pose:
            robot.update_pose(msg)
        elif msg.topic == topic_order:
            robot.receive_order(msg)
        elif msg.topic == topic_loc_pose:
            robot.robot_fine(msg)


    client = mqtt.Client()
    client.on_connect = on_connect 
    client.on_message = on_message 
    client.connect("172.22.222.238", 1883, 60)
    client.loop_start()

    last_status_time = time.time()
    
    while True:
        if robot.state=="docking_complete":
            # client.publish(topic_stop_fine, json.dumps({"value":True}))
            robot.state="conveyer_on"
            # print("published to the topic")
            
        if robot.state=="conveyer_on" and robot.trajectory[robot.current_index-1].get("action") is not None:
            print("conveyer")
            convey(robot,client,topic_conveyer_robot,robot.con_speed)
            robot.state= "undocking"
        else:
            robot.state= "undocking"
        
        
        

        if robot.fine_loc_trigger:
            client.publish(topic_loc, json.dumps(robot.fine_loc_trigger))
            robot.fine_loc_trigger = None  # Clear the trigger
            robot.docking_active = True    # <--- Lock the robot into docking mode
            # print("shit")

        # 2. Decide which controller to use (Happens CONTINUOUSLY)
        if robot.docking_active:
            cmd = docking(robot)
        elif robot.undocking_active:
            cmd = undocking(robot,topic_stop_fine,client)
        else:
            cmd = follow_trajectory(robot)
            robot.state= "path_following"
            # print("trajectory")


        # 3. Publish the velocity command
        client.publish(topic_cmd, json.dumps(cmd))
       
        # Send status update if an event occurred or periodically (every 30s)
        if robot.status_update_needed or time.time() - last_status_time >= 30:
            send_status_update(client, topic_state, robot)
            robot.status_update_needed = False
            last_status_time = time.time()
        
        time.sleep(0.1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default="cat001") 
    args = parser.parse_args()
    main(args.robot)



