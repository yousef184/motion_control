'''
Integrated VDA5050 Robot Controller
Combines PD differential control with robust pose initialization and angle normalization.
Fully compliant with VDA 5050 V2.1.0 State Schema with strict node/edge lifecycle management.
Implements Dual-Mode Safety Logic.

Fixes (Fleet-Management-Kompatibilität):
  Fix 1 – Undocking-Ziel: wird jetzt beim Docking-Abschluss fest gespeichert.
           Vorher: trajectory[current_index - 1] → nach neuer Order (current_index=0)
           zeigt trajectory[-1] auf den letzten Knoten der NEUEN Trajectory → Robot
           fährt in die falsche Richtung.
  Fix 2 – Process-Aktionen: werden jetzt nach Ablauf von processingTime auf FINISHED
           gesetzt. Vorher blieben sie dauerhaft auf RUNNING → Fleet Manager hing
           endlos auf Process-Legs.
  Fix 3 – Kein vorzeitiges FINISHED-Signal: convey() sendet den Status mit
           pick/drop=FINISHED jetzt erst NACH abgeschlossenem Undocking.
           Vorher wurde er mitten in convey() geschickt → Fleet Manager startete die
           nächste Leg, während der Roboter noch an der Station stand, was zu
           Race Conditions auf agent.current_task führte (Skip-Bug).
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
    try:
        with open(os.path.normpath(_STATE_SCHEMA_PATH), "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}

_STATE_SCHEMA = _load_state_schema()

C2 = 0.045
C1A = 0.03
M1A = 0.052
M2 = 0.55
class Robot:
    def __init__(self, name):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # VDA 5050 Tracking Variables
        self.last_node_id = ""
        self.last_node_sequence_id = 0
        self.current_order_id = ""
        self.current_order_update_id = 0

        self.nodes = []
        self.edges = []
        self.action_states = []
        self.trajectory = []
        self.current_index = 0          
        
        # PD Controller tracking variables
        self.prev_distance_error = 0.0  
        self.prev_angle_error = 0.0
        self.sum_distance_error = 0.0   
        self.sum_angle_error = 0.0      
        self.last_time = time.time()   

        # Filter and State variables
        self.pose_initialized = False
        self.x_f = 0.0
        self.y_f = 0.0
        self.theta_f = 0.0
        self.alpha = 0.55               
        
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
        
        self.con_speed = 0.2
        self.sensor_a = False
        self.sensor_br = False
        self.sensor_ar = False

        self.undocking_target = None

        # Safety Tracker
        self.safety_status = 0

        # FIX 1: Undocking-Ziel wird beim Docking-Abschluss gespeichert,
        # damit receive_order (current_index=0) es nicht korrumpieren kann.
        self.undocking_target = None  # (x, y) – gesetzt in docking() wenn DOCKED

        # FIX 2: Timer für Process-Aktionen.
        # Tupel (action_id, finish_time) oder None.
        self.pending_process_action = None

        # FIX 3: Status mit FINISHED erst nach Undocking senden.
        # Wird in convey() gesetzt, ausgelöst am Ende von undocking().
        self.send_status_after_undock = False

    def update_pose(self, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        
        pos = payload.get("position", {})
        raw_x = pos.get("x", self.x_f)
        raw_y = pos.get("y", self.y_f)
        
        ori = payload.get("orientation", {})
        z = ori.get("z", 0.0)
        w = ori.get("w", 1.0)
        raw_theta = 2.0 * math.atan2(z, w)

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

    def update_safety(self, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        self.safety_status = payload.get("safety_status", 0)
    
    def receive_order(self, msg):
        print(self.trajectory)
     
        payload = json.loads(msg.payload.decode("utf-8"))

        incoming_order_id = str(payload.get("orderId", ""))
        is_order_update = (
            bool(self.current_order_id)
            and incoming_order_id == self.current_order_id
        )
        previous_action_states = {
            action_state["actionId"]: dict(action_state)
            for action_state in self.action_states
            if action_state.get("actionId")
        }

        self.current_order_id = incoming_order_id
        self.current_order_update_id = payload.get("orderUpdateId", 0)
        
        raw_nodes = payload.get("nodes", [])
        raw_edges = payload.get("edges", [])
    
        self.nodes = [n for n in raw_nodes if n.get("released", False)]
        self.edges = [e for e in raw_edges if e.get("released", False)]

        for node in self.nodes:
            sequence_id = node.get("sequenceId")
            if not isinstance(sequence_id, int) or sequence_id % 2 != 0:
                print(f"[ORDER WARNING] Node has invalid sequenceId: {node}")

        for edge in self.edges:
            sequence_id = edge.get("sequenceId")
            if not isinstance(sequence_id, int) or sequence_id % 2 != 1:
                print(f"[ORDER WARNING] Edge has invalid sequenceId: {edge}")

        action_states_by_id = previous_action_states if is_order_update else {}
        for n in self.nodes:
            for a in n.get("actions", []):
                action_id = a.get("actionId", "")
                if action_id and action_id not in action_states_by_id:
                    action_states_by_id[action_id] = {
                        "actionId": action_id,
                        "actionType": a.get("actionType", ""),
                        "actionStatus": "WAITING",
                    }
        for e in self.edges:
            for a in e.get("actions", []):
                action_id = a.get("actionId", "")
                if action_id and action_id not in action_states_by_id:
                    action_states_by_id[action_id] = {
                        "actionId": action_id,
                        "actionType": a.get("actionType", ""),
                        "actionStatus": "WAITING",
                    }
        self.action_states = list(action_states_by_id.values())
    
        # Trajectory aufbauen
        self.trajectory = []
        for n in self.nodes:
            pos = n.get("nodePosition", {})
            x, y = pos.get("x"), pos.get("y")
    
            if x is None or y is None:
                continue
                
            docking_action = None
            for action in n.get("actions", []):
                if action.get("actionType") in ("init_fine_positioning", "pick", "drop"):
                    docking_action = action.get("actionType")
                    break
            
            waypoint = {
                "nodeId": n.get("nodeId"),
                "sequenceId": n.get("sequenceId", 0),
                "x": x,
                "y": y,
                "is_docking_node": False,
                "is_internal": False,
                "action": docking_action,
                "actions": n.get("actions", [])
            }
            self.trajectory.append(waypoint)

            if docking_action == "init_fine_positioning":
                fine_position_actions = [
                    action for action in n.get("actions", [])
                    if action.get("actionType") == "init_fine_positioning"
                ]
                params = {
                    p["key"]: p["value"]
                    for p in fine_position_actions[0].get("actionParameters", [])
                }
        
                station_x = float(params.get("init_fine_pos_x", 0.0))
                station_y = float(params.get("init_fine_pos_y", 0.0))
                station_theta = float(params.get("init_fine_pos_theta", 0.0))
        
                pre_dock_distance = 0.5
                station_id = params.get("init_fine_pos_name")
               
                if station_id is not None:
                    if "001_A" in station_id and self.name == "mouse001":
                        pre_x = 5.7
                        pre_y = 4.00
                        theta = 1.57

                    elif "001_B" in station_id and self.name == "mouse001":
                        pre_x = 6.97
                        pre_y = 4.00
                        theta = 1.57

                    elif "001_A" in station_id and self.name == "cat001":
                        theta = float(params.get("fine_pos_control_theta", 0.0))
                        pre_x = station_x - pre_dock_distance * math.cos(theta) + float(params.get("fine_pos_control_x", 0.0))
                        pre_y = station_y - pre_dock_distance * math.sin(theta) + float(params.get("fine_pos_control_y", 0.0))

                    elif "001_B" in station_id and self.name == "cat001":
                        pre_x = 7.64
                        pre_y = 4.73
                        theta = 3.14

                    elif "002_A" in station_id and self.name == "mouse001":
                        pre_x = 1.45
                        pre_y = 1.35
                        theta = 0

                    elif "002_A" in station_id and self.name == "cat001":
                        pre_x = 2.23
                        pre_y = 1.96
                        theta = -1.57
                        print("cool")
                    else:
                        theta = float(params.get("fine_pos_control_theta", 0.0))
                        pre_x = station_x - pre_dock_distance * math.cos(theta) + float(params.get("fine_pos_control_x", 0.0))
                        pre_y = station_y - pre_dock_distance * math.sin(theta) + float(params.get("fine_pos_control_y", 0.0))

                pre_dock_wp = {
                    "nodeId": f"{n.get('nodeId')}_pre_dock",
                    "sequenceId": n.get("sequenceId", 0),
                    "x": pre_x,
                    "y": pre_y,
                    "is_docking_node": True,
                    "is_internal": True,
                    "action": "init_fine_positioning",
                    "station_id": params.get("init_fine_pos_name"),
                    "station_x": station_x,
                    "station_y": station_y,
                    "station_theta": station_theta,
                    "localx": float(params.get("fine_pos_control_x", 0.0)),
                    "localy": float(params.get("fine_pos_control_y", 0.0)),
                    "localt": theta,
                    "actions": fine_position_actions
                }
                self.trajectory.append(pre_dock_wp)
            
        self.current_index = 0
        self.status_update_needed = True

        # FIX 2: Process-Timer bei neuer Order zurücksetzen, da die Action-ID
        # einer alten Order ungültig wird.
        self.pending_process_action = None

        print("[ORDER] trajectory built.")
        print(self.trajectory)

    def robot_fine(self, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        self.station_detected = payload.get("station_detected", 0)             
        if self.station_detected > 0:
            self.fine_pose = payload.get("pose")    

    def build_status_message(self):
        status = {
            "headerId": int(time.time() * 10) % 1000000,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "version": "V2.1.0",
            "manufacturer": "IFL",
            "serialNumber": self.name,
            "orderId": self.current_order_id,
            "orderUpdateId": self.current_order_update_id,
            "lastNodeId": self.last_node_id,
            "lastNodeSequenceId": self.last_node_sequence_id,
            "nodeStates": self.nodes,
            "edgeStates": self.edges,
            "zoneSetId": "DEFAULT_ZONE",
            "agvPosition": {
                "positionInitialized": self.pose_initialized,
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "mapId": "Map_1"
            },
            "driving": self.current_index < len(self.trajectory) and (self.cmd_linear_v != 0.0 or self.cmd_angular_v != 0.0),
            "actionStates": self.action_states,
            "batteryState": {
                "batteryCharge": 80,
                "charging": False
            },
            "operatingMode": "AUTOMATIC",
            "errors": [],
            "safetyState": {
                "eStop": "NONE",
                "fieldViolation": False,
                "safetyState": self.safety_status
            }
        }
        return status

def ramp_velocity(desired, current, max_acc, dt):
    max_delta = max_acc * dt
    delta = desired - current

    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta

    return current + delta

def calculate_relative_pose(robot_x, robot_y, robot_theta, station_x, station_y, station_theta):
    dx = robot_x - station_x
    dy = robot_y - station_y
    
    x_rel = dx * math.cos(station_theta) + dy * math.sin(station_theta)
    y_rel = -dx * math.sin(station_theta) + dy * math.cos(station_theta)
    
    theta_rel = robot_theta - station_theta
    while theta_rel > math.pi: 
        theta_rel -= 2.0 * math.pi
    while theta_rel < -math.pi: 
        theta_rel += 2.0 * math.pi
        
    return x_rel, y_rel, theta_rel


def set_action_status(robot: Robot, action_id: str, status: str) -> bool:
    """Setzt eine VDA-Action auf neuen Status, ohne terminalen Status zurückzusetzen."""
    for action_state in robot.action_states:
        if action_state.get("actionId") != action_id:
            continue

        current_status = action_state.get("actionStatus")
        if current_status in {"FINISHED", "FAILED"} and current_status != status:
            return False
        if current_status == status:
            return False

        action_state["actionStatus"] = status
        robot.status_update_needed = True
        return True

    print(f"[ACTION WARNING] Unknown actionId: {action_id}")
    return False


def set_target_actions_status(robot: Robot, target: dict, status: str) -> int:
    updated_actions = 0
    for action in target.get("actions", []):
        action_id = action.get("actionId")
        if action_id and set_action_status(robot, action_id, status):
            updated_actions += 1
    return updated_actions


def mark_vda_node_reached(robot: Robot, target: dict) -> None:
    """Meldet Ankunft an einem echten Order-Knoten und entfernt das abgefahrene Präfix."""
    if target.get("is_internal", False):
        return

    node_id = target.get("nodeId", "")
    sequence_id = target.get("sequenceId")
    if not node_id or not isinstance(sequence_id, int):
        print(f"[STATE WARNING] Cannot report invalid node target: {target}")
        return

    robot.last_node_id = node_id
    robot.last_node_sequence_id = sequence_id
    robot.nodes = [
        node for node in robot.nodes
        if node.get("sequenceId", -1) > sequence_id
    ]
    robot.edges = [
        edge for edge in robot.edges
        if edge.get("sequenceId", -1) > sequence_id
    ]
    robot.status_update_needed = True


def follow_trajectory(robot: Robot):
    if robot.current_index < len(robot.trajectory):
        target = robot.trajectory[robot.current_index]
        target_x = target['x']
        target_y = target['y']
        current_node_id = target.get("nodeId", "")

        # Stationsknoten (N1A, N1B) überspringen — Docking/Conveyor-Flow hat
        # sie bereits behandelt. Beim Bypass wird mark_vda_node_reached aufgerufen,
        # damit nodeStates korrekt aktualisiert wird.
        if "A" in current_node_id or "B" in current_node_id:
            robot.current_index += 1
            
            robot.prev_distance_error = 0.0
            robot.prev_angle_error = 0.0
            robot.sum_distance_error = 0.0
            robot.sum_angle_error = 0.0
            robot.cmd_linear_v = 0.0
            robot.cmd_angular_v = 0.0
            mark_vda_node_reached(robot, target)
            return {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }


        kp1, ki1, kd1 = 0.5, 0.05, 0.05   
        kp2, ki2, kd2 = 1.1, 0.10, 0.1     

        MAX_I_LIN = 2.0
        MAX_I_ANG = 2.0

        current_time = time.time()
        dt = current_time - robot.last_time
        if dt <= 0.0:
            dt = 0.001 

        dx = target_x - robot.x
        dy = target_y - robot.y
        
        distance_error = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        angle_error = target_heading - robot.theta
        
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        if abs(angle_error) <= 0.35:
            robot.sum_distance_error += distance_error * dt
            robot.sum_distance_error = max(-MAX_I_LIN, min(MAX_I_LIN, robot.sum_distance_error))

        robot.sum_angle_error += angle_error * dt
        robot.sum_angle_error = max(-MAX_I_ANG, min(MAX_I_ANG, robot.sum_angle_error))

        dv = (distance_error - robot.prev_distance_error) / dt
        da = (angle_error - robot.prev_angle_error) / dt

        linear_vel = kp1 * distance_error + ki1 * robot.sum_distance_error + kd1 * dv
        angular_vel = kp2 * angle_error + ki2 * robot.sum_angle_error + kd2 * da

        if abs(angle_error) > 0.35:
            linear_vel = 0.0

        MAX_LIN_ACC, MAX_ANG_ACC = 0.25, 0.8
        MAX_LIN_VEL, MAX_ANG_VEL = 0.32, 0.75
        
        desired_linear = max(-MAX_LIN_VEL, min(MAX_LIN_VEL, linear_vel))
        desired_angular = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, angular_vel))   
        
        linear_vel = ramp_velocity(desired_linear, robot.cmd_linear_v, MAX_LIN_ACC, dt)
        angular_vel = ramp_velocity(desired_angular, robot.cmd_angular_v, MAX_ANG_ACC, dt)

        robot.cmd_linear_v = linear_vel
        robot.cmd_angular_v = angular_vel
        robot.prev_distance_error = distance_error
        robot.prev_angle_error = angle_error
        robot.last_time = current_time

        ARRIVAL_THRESHOLD = 0.1
        if distance_error < ARRIVAL_THRESHOLD:
            
            if target.get("is_docking_node") == True:
                mps_x = target.get("station_x", 0.0)
                mps_y = target.get("station_y", 0.0)
                mps_theta = target.get("station_theta", 0.0)
                
                spr_x, spr_y, spr_theta = calculate_relative_pose(
                    robot.x, robot.y, robot.theta, mps_x, mps_y, mps_theta
                )
                
                robot.fine_loc_trigger = {
                    "x": spr_x,
                    "y": spr_y,
                    "theta": spr_theta,
                    "station_id": target.get("station_id", "")
                }

            if not target.get("is_internal", False):
                mark_vda_node_reached(robot, target)
                set_target_actions_status(robot, target, "RUNNING")

                # FIX 2: Process-Aktion planen.
                # processingTime kommt als direktes Feld in der Action-Dict
                # (so wie unser Fleet Manager es in _build_leg_nodes setzt).
                for action in target.get("actions", []):
                    if action.get("actionType") == "process":
                        processing_time = float(action.get("processingTime", 0.0))
                        robot.pending_process_action = (
                            action.get("actionId"),
                            time.time() + processing_time
                        )
                        print(f"[PROCESS] Aktion {action.get('actionId')} geplant, "
                              f"Fertig in {processing_time}s.")
                        break

            robot.current_index += 1
            robot.status_update_needed = True
            
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
        robot.cmd_linear_v = 0.0
        robot.cmd_angular_v = 0.0
        return {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

def docking(robot: Robot):
    target = robot.trajectory[robot.current_index - 1]
    t_x = target.get("localx", 0.0)
    t_y = target.get("localy", 0.0)
    t_theta = target.get("localt", 0.0)

    if not getattr(robot, 'fine_pose', None) or robot.station_detected == 0:
        s_x = target.get("station_x", 0.0)
        s_y = target.get("station_y", 0.0)
        s_theta = target.get("station_theta", 0.0)
        
        c_x, c_y, c_theta = calculate_relative_pose(
            robot.x, robot.y, robot.theta, 
            s_x, s_y, s_theta
        )
    else:
        c_x = robot.fine_pose["x"]
        c_y = robot.fine_pose["y"]
        c_theta = robot.fine_pose["theta"]

    dx = t_x - c_x
    dy = t_y - c_y
    dist_error = math.hypot(dx, dy)
    
    def norm_angle(a):
        while a > math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    final_angle_err = norm_angle(t_theta - c_theta)

    MAX_LIN = 0.05  
    MAX_ANG = 0.15  
    error = 0.55
    # if "002" in target["station_id"] and "cat" in robot.name:
    #     error = C2
    # else: 
    #     error = 0.04
    
    lin_vel, ang_vel = 0.0, 0.0
    
    if not hasattr(robot, 'docking_step'):
        robot.docking_step = "TURN_1"

    if robot.docking_step == "TURN_1":
        if abs(final_angle_err) > 0.03:
            ang_vel = max(-MAX_ANG, min(MAX_ANG, final_angle_err * 0.8))
        else:
            robot.docking_step = "MOVE"

    elif robot.docking_step == "MOVE":
        if abs(final_angle_err) > 0.03:
            robot.docking_step = "TURN_1"
        elif dist_error > error:
            lin_vel = max(-MAX_LIN, min(MAX_LIN, dist_error * 1))
            ang_vel = max(-MAX_ANG, min(MAX_ANG, final_angle_err * 0.8))
        else:
            print("\n✅ [DOCKING] SUCCESSFULLY DOCKED WITHIN TOLERANCES!\n")
            robot.docking_step = "TURN_1" 
            robot.docking_active = False  
            robot.status_update_needed = True 
            robot.undocking_active = True
            robot.state = "docking_complete"
            
            # --- LOCK THE TARGET BEFORE MQTT CAN OVERWRITE IT ---
            robot.undocking_target = (target.get("x", 0.0), target.get("y", 0.0))

    if dist_error < robot.min_err:
        robot.min_err = dist_error
    
    print(dist_error)
        
    return {
        "linear": {"x": lin_vel, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": ang_vel}
    }


def undocking(robot, topic, client):
    if not hasattr(robot, "undocking_active"):
        robot.undocking_active = True

    MAX_LIN = 0.05
    UNDOCK_DIST = 0.1

    # 1. Safely use the locked coordinates
    if hasattr(robot, "undocking_target") and robot.undocking_target is not None:
        t_x, t_y = robot.undocking_target
    else:
        # Absolute fallback if the target was somehow lost
        target = robot.trajectory[robot.current_index - 1]
        t_x = target.get("x", 0.0)
        t_y = target.get("y", 0.0)

    dx = t_x - robot.x
    dy = t_y - robot.y
    dist_error = math.hypot(dx, dy)

    # 2. Check if undocking is complete
    if dist_error <= UNDOCK_DIST:
        print("\n🔓 [UNDOCKING] COMPLETE\n")
        robot.undocking_active = False
        robot.undocking_target = None # Clear it for the next run
        client.publish(topic, json.dumps({"value": True}))
        
        # 3. Fire off the delayed FINISHED status to the Fleet Manager
        if getattr(robot, "send_status_after_undock", False):
            robot.send_status_after_undock = False
            robot.status_update_needed = True 
            print("[UNDOCKING] Sending delayed FINISHED status to Fleet Manager.")

        return {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
        
    # 3. Continue driving backward
    print(dist_error)
    lin_vel = -min(MAX_LIN, dist_error * 0.8)
    return {
        "linear": {"x": lin_vel, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
    }

def con_move(robot, client, topicR, topicS, speed, target_sensor):
    station_speed = speed
    if "mouse" in topicR:
        station_speed = -speed
    client.publish(topicR, json.dumps({"value": speed}))
    client.publish(topicS, json.dumps({"value": station_speed}))
    
    flag = 0
    
    while True:
        sensor_value = getattr(robot, target_sensor)
        
        client.publish(topicR, json.dumps({"value": speed}))
        client.publish(topicS, json.dumps({"value": station_speed}))
        time.sleep(0.1) 
        
        if sensor_value == True and flag == 0:
            flag = 1
            print(1)
        elif sensor_value == False and flag == 1:
            flag = 2
            print(2)
        
        if flag == 2:
            break

    client.publish(topicR, json.dumps({"value": 0.0}))
    client.publish(topicS, json.dumps({"value": 0.0}))

def convey(robot, client, topic, state_topic, speed):
    """
    FIX 3: Das finale send_status_update (pick/drop=FINISHED) wurde entfernt.
    Stattdessen wird robot.send_status_after_undock = True gesetzt. Der Status
    wird erst nach Abschluss des Undockings gesendet (in undocking()). Das
    verhindert, dass der Fleet Manager die nächste Leg dispatcht, während der
    Roboter noch an der Station steht (Race Condition auf agent.current_task).
    """
    if robot.current_index >= len(robot.trajectory):
        raise RuntimeError("Conveyor action requested without a target waypoint")

    docking_target = robot.trajectory[robot.current_index - 1]
    action_target = robot.trajectory[robot.current_index]
    actions = action_target.get("actions", [])
    if not actions:
        raise RuntimeError("Conveyor target does not contain a VDA action")

    station_name = docking_target["station_id"]
    action_type = actions[0]["actionType"]

    # Ankunft und RUNNING melden, bevor der blockierende Förderband-Loop startet.
    mark_vda_node_reached(robot, action_target)
    set_target_actions_status(robot, action_target, "RUNNING")
    send_status_update(client, state_topic, robot)
    robot.status_update_needed = False

    station_topic = f"KIT/IMRL/{station_name[0:-2]}/conveyor/speed"
    print(action_type, station_topic)

    try:
        if "_A" in station_name and action_type == "drop":
            if "mouse" in robot.name:
                con_move(robot, client, topic, station_topic, -speed, "sensor_a")
            else:
                con_move(robot, client, topic, station_topic, speed, "sensor_a")
        elif "_B" in station_name and action_type == "pick":
            if "mouse" in robot.name:
                con_move(robot, client, topic, station_topic, -speed, "sensor_ar")
            else:
                con_move(robot, client, topic, station_topic, speed, "sensor_br")
        elif "_B" in station_name and action_type == "drop":
            if "mouse" in robot.name:
                con_move(robot, client, topic, station_topic, speed, "sensor_a")
            else:
                con_move(robot, client, topic, station_topic, -speed, "sensor_a")
        elif "_A" in station_name and action_type == "pick":
            if "mouse" in robot.name:
                con_move(robot, client, topic, station_topic, speed, "sensor_ar")
            else:
                con_move(robot, client, topic, station_topic, -speed, "sensor_br")
            
    except Exception:
        set_target_actions_status(robot, action_target, "FAILED")
        send_status_update(client, state_topic, robot)
        robot.status_update_needed = False
        raise

    set_target_actions_status(robot, action_target, "FINISHED")

    # FIX 3: Status NICHT sofort senden — Fleet Manager soll erst nach
    # abgeschlossenem Undocking die FINISHED-Meldung erhalten.
    robot.send_status_after_undock = True
    robot.status_update_needed = False

def apply_safety_filter(robot: Robot, cmd, is_fine_mode: bool):
    """
    Safety-Regeln abhängig von Normal- oder Fine-Mode.
    Normal Mode: 1: langsam, >=2: Stop.
    Fine Mode: 3: langsam, >=4: Stop.
    """
    safety_status = robot.safety_status

    if not is_fine_mode:
        if safety_status == 1:
            cmd["linear"]["x"] *= 0.4
            cmd["angular"]["z"] *= 0.5
        elif safety_status >= 2:
            cmd["linear"]["x"] = 0.0
            cmd["angular"]["z"] = 0.0
            robot.prev_distance_error = 0.0
            robot.prev_angle_error = 0.0
            robot.sum_distance_error = 0.0
            robot.sum_angle_error = 0.0
            robot.cmd_linear_v = 0.0
            robot.cmd_angular_v = 0.0
    else:
        if safety_status == 3:
            cmd["linear"]["x"] *= 0.4
            cmd["angular"]["z"] *= 0.5
        elif safety_status >= 4:
            cmd["linear"]["x"] = 0.0
            cmd["angular"]["z"] = 0.0
            
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

    topic_cmd = f"KIT/IMRL/{robot_name}/cmd"
    topic_pose = f"KIT/IMRL/{robot_name}/pose"
    topic_order = f"KIT/IMRL/{robot_name}/order"
    topic_state = f"KIT/IMRL/{robot_name}/state"
    topic_safety = f"KIT/IMRL/{robot_name}/safety"
    topic_loc = f"KIT/IMRL/{robot_name}/fine_loc/start"
    topic_loc_pose = f"KIT/IMRL/{robot_name}/fine_loc/pose"
    topic_stop_fine = f"KIT/IMRL/{robot_name}/fine_loc/stop"
    topic_conveyer_robot = f"KIT/IMRL/{robot_name}/conveyor/speed"
    topic_coneryer_states = f"KIT/IMRL/+/conveyor/state"


    def on_connect(client, userdata, flags, rc): 
        client.subscribe(topic_pose)
        client.subscribe(topic_order)
        client.subscribe(topic_loc_pose)
        client.subscribe(topic_coneryer_states)
        client.subscribe(topic_safety) # Subscribed to safety node
        
    def on_message(client, userdata, msg):
        if msg.topic == topic_pose:
            robot.update_pose(msg)
        elif msg.topic == topic_order:
            robot.receive_order(msg) # Removed robot parameter argument
        elif msg.topic == topic_loc_pose:
            robot.robot_fine(msg)
        elif msg.topic == topic_safety:
            robot.update_safety(msg) # Parsing the safety status integer
        elif "/station001/conveyor/state" in msg.topic or "/station002/conveyor/state" in msg.topic:
            # print("shit1")
            payload = json.loads(msg.payload.decode('utf-8'))
            robot.sensor_a = payload.get("sensor_a", False)
        elif f"/{robot_name}/conveyor/state" in msg.topic:
            # print("shit2")
            payload = json.loads(msg.payload.decode('utf-8'))
            robot.sensor_br = payload.get("sensor_b", False)
            robot.sensor_ar = payload.get("sensor_a", False)
            
    client = mqtt.Client()
    client.on_connect = on_connect 
    client.on_message = on_message 
    client.connect("172.22.222.238", 1883, 60)
    client.loop_start()

    last_status_time = time.time()
    
    while True:
        # FIX 2: Process-Aktion abschließen, wenn processingTime abgelaufen.
        # Setzt actionStatus auf FINISHED und löst einen Status-Update aus,
        # damit der Fleet Manager das Leg als erledigt erkennt.
        if robot.pending_process_action is not None:
            action_id, finish_time = robot.pending_process_action
            if time.time() >= finish_time:
                set_action_status(robot, action_id, "FINISHED")
                robot.pending_process_action = None
                robot.status_update_needed = True
                print(f"[PROCESS] Aktion {action_id} abgeschlossen → FINISHED.")

        if robot.state == "docking_complete":
            robot.state = "conveyer_on"
            
        if robot.state == "conveyer_on":
            if robot.current_index < len(robot.trajectory):
                current_action = robot.trajectory[robot.current_index].get("action")
                if current_action in ["pick", "drop"]:
                    convey(
                        robot,
                        client,
                        topic_conveyer_robot,
                        topic_state,
                        robot.con_speed,
                    )
                    robot.state = "undocking"
                    print("[CONVEYOR] Aktion abgeschlossen, Undocking startet.")
        
        if robot.fine_loc_trigger:
            client.publish(topic_loc, json.dumps(robot.fine_loc_trigger))
            robot.fine_loc_trigger = None  
            robot.docking_active = True    

        if robot.docking_active:
            cmd = docking(robot)
            is_fine_mode = True
        elif robot.undocking_active:
            cmd = undocking(robot, topic_stop_fine, client)
            is_fine_mode = True
        elif robot.state == "conveyer_on":
            cmd = {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
            is_fine_mode = False
        else:
            cmd = follow_trajectory(robot)
            robot.state = "path_following"
            is_fine_mode = False

        # cmd = apply_safety_filter(robot, cmd, is_fine_mode)
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