'''
This script is an example code to be used for individual task 1. 
'''

import json
from platform import node
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
        - Decode the JSON payload  ---1
        - Extract 'nodes' and 'edges' from the message ---2
        - Filter only the 'released' ones ---3 
        - Store relevant information in self.nodes and self.edges ---4

        Example (to be implemented):
        # for node in nodes:
        #     print(f"Node ID: {node['nodeId']} at ({node['x']}, {node['y']})")
        """
        # TODO: Implement order parsing and printing logic here
        payload = json.loads(msg.payload.decode("utf-8")) #1

        nodes = payload.get("nodes", []) #2
        edges = payload.get("edges", []) #2

        self.nodes = [n for n in nodes if n.get("released", False)] #3
        self.edges = [e for e in edges if e.get("released", False)] #3

    route = []  #4  并且打印出每个节点的ID和坐标信息
    for node in self.nodes:
        node_id = node.get("nodeId")
        pos = node.get("nodePosition", {})
        x = pos.get("x")
        y = pos.get("y")

    if x is not None and y is not None:
        route.append((x, y))
        print(f"nodeId={node_id}, x={x}, y={y}")
    else:
        print(f"nodeId={node_id}, nodePosition missing x/y: {pos}")

    print("Released route:", route)


    pass

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
