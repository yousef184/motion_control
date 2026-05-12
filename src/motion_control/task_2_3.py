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

_STATE_SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "data", "interface", "state.schema"
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
        pass

    def receive_order(self, msg):
        # TODO: Parse nodes and edges from the order message
        # TODO: Generate trajectory based on node positions
        self.trajectory = []       # Replace with generated waypoints
        self.current_index = 0     # Reset index
        self.last_node_id = None
        self.status_update_needed = True  # Order receipt triggers a status update

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
        status = {}
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
        target = robot.trajectory[robot.current_index]

        # TODO: Compute control commands (linear_vel, angular_vel)
        linear_vel = 0.0
        angular_vel = 0.0

        # TODO: Check if the robot is close enough to the target
        # If so:
        # - robot.current_index += 1
        # - robot.last_node_id = ...
        # - robot.status_update_needed = True

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

    topic_cmd = f"uagv/v2/KIT/{robot_name}/cmd"
    topic_pose = f"uagv/v2/KIT/{robot_name}/pose"
    topic_order = f"uagv/v2/KIT/{robot_name}/order"
    topic_state = f"uagv/v2/KIT/{robot_name}/state"

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
