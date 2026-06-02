import json
import os
from datetime import datetime, timezone
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher

_FLEET_MANAGEMENT_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")
)


class OrderInterface:
    def __init__(self, config_data: dict, logging: object, order_topic: str, agentId: str) -> None:
        self.config_data = config_data
        self.order_topic = order_topic
        self.logging = logging
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=order_topic,
                                            client_id=f'order_publisher_agent_{self.agentId}', logging=self.logging)

    def generate_order_message(self, agent: object, orderId: str, order_updateId: int,
                               nodes: list, edges: list) -> None:
        # TODO Task 3: Generate the VDA5050 order message automatically based on the passed parameters.
        #
        # Suggested steps:
        #   1. Build nodes_msg: for each node in 'nodes', create a dict with nodeId, sequenceId,
        #      released, nodePosition (x, y, mapId, and theta only if not None), and actions.
        #   2. Build edges_msg: for each edge in 'edges', create a dict with edgeId, sequenceId,
        #      released, startNodeId, endNodeId, and actions (empty list).
        #   3. Assemble order_msg with headerId, timestamp (ISO 8601), version, manufacturer,
        #      serialNumber, orderId, orderUpdateId, nodes_msg, and edges_msg.
        #   4. Publish via self.mqtt_publisher.publish(order_msg, qos=0).
        #   5. Increment agent.agents.order_header_id.
        #
        # For now, the hardcoded example order message is published instead:
        # order_msg_path = os.path.join(_FLEET_MANAGEMENT_DIR, "data", "input_files", "orderMessage_Example.json")
        # with open(order_msg_path, 'r') as order_msg_file:
        #     order_msg = json.load(order_msg_file)
        # self.mqtt_publisher.publish(order_msg, qos=0)

        nodes_msg = []
        for i, node in enumerate(nodes):
            node_position = {
                "x": node["x"],
                "y": node["y"],
                "mapId": "Map_1",
            }
            if node.get("theta") is not None:
                node_position["theta"] = node["theta"]
            nodes_msg.append({
                "nodeId": node["nodeId"],
                "sequenceId": i * 2,
                "released": True,
                "nodePosition": node_position,
                "actions": node.get("actions", []),
            })

        edges_msg = []
        for i, edge in enumerate(edges):
            edges_msg.append({
                "edgeId": edge["edgeId"],
                "sequenceId": i * 2 + 1,
                "released": True,
                "startNodeId": edge["startNodeId"],
                "endNodeId": edge["endNodeId"],
                "actions": edge.get("actions", []),
            })

        order_msg = {
            "headerId": agent.agents.order_header_id,
            "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "version": "V2.1.0",
            "manufacturer": "IFL",
            "serialNumber": f"IFL_{agent.agentId}",
            "orderId": orderId,
            "orderUpdateId": order_updateId,
            "nodes": nodes_msg,
            "edges": edges_msg,
        }

        self.mqtt_publisher.publish(order_msg, qos=0)
        agent.agents.order_header_id += 1
