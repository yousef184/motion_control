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
        # TODO: Implement order parsing and printing logic here
        pass

def on_connect(client, userdata, flags, rc):
    client.subscribe(userdata["topic_order"])

def on_message(client, userdata, msg):
    robot: Robot = userdata["robot"]

    if msg.topic == userdata["topic_order"]:
        robot.receive_order(msg)

def main():
    robot_name = "mouse001"
    topic_order = f"uagv/v2/KIT/{robot_name}/order"

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
