import json
from vda5050_interface.mqtt_clients.mqtt_publisher import MQTTPublisher


class OrderInterface:
    def __init__(self, config_data:dict, logging:object, order_topic:str, agentId:str) -> None:
        """
        Initialize the OrderInterface class.

        :param config_data: Data from the configuration file.
        :param logging: Logging object.
        :param order_topic: MQTT topic for the order messages.
        :param agentId: The ID of the agent.
        """
        self.config_data = config_data
        self.order_topic = order_topic
        self.logging = logging
        self.agentId = agentId
        self.mqtt_publisher = MQTTPublisher(config_data=config_data, channel=order_topic,
                                            client_id=f'order_publisher_agent_{self.agentId}', logging=self.logging)

    def generate_order_message(self, agent:object, orderId:str, order_updateId:int, nodes:list, edges:list) -> None:
        """
        Generate the VDA5050 order message for the real agents.
        
        :param agent: Agent object.
        :param fleet_management: FleetManagement object.
        :param orderId: Order ID of the VDA5050 order.
        :param order_updateId: Order update ID of the VDA5050 order.
        :param nodes: List of nodes of the VDA5050 order.
        :param edges: List of edges of the VDA5050 order.
        """
        # TODO: Generate the order message automatically based on the passed parameters.

        # Load the example order message.
        order_msg_path = "data/input_files/orderMessage_Example.json"
        with open(order_msg_path, 'r') as order_msg_file:
            order_msg = json.load(order_msg_file)

        # Publish the order message.
        self.mqtt_publisher.publish(order_msg, qos=0)
