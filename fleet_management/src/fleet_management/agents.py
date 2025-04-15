# Import necessary libraries and modules.
import math
import json
from vda5050_interface.mqtt_clients.mqtt_subscriber import MQTTSubscriber
from vda5050_interface.interfaces.order_interface import OrderInterface


class Agents:
    """
    Class representing the digital twins of the agents controlled by the fleet manager.
    """
    def __init__(self, config_data, graph, agents_initialization_data, logging, simulation_start_time) -> None:
        """
        Initialize the Agents object.

        :param config_data: The data from the configuration file.
        :param graph: The graph object based on which the agents are controlled by the fleet manager.
        :param agents_initialization_data: The data from the agents initialization file.
        :param logging: The logging object.
        :param simulation_start_time: The start time of the simulation.
        """
        self.simulation_start_time = simulation_start_time
        self.logging = logging
        self.config_data = config_data
        self.graph = graph
        self.order_header_id = 1
        self.agents = self.get_agents(agents_initialization_data)

    def get_agents(self, agents_initialization_data) -> list:
        """
        Create the digital twins of the agents based on the data from the agents initialization file.
        
        :param agents_initialization_data: The data from the agents initialization file.
        :return: A list of digital twin agent objects.
        """
        # TODO: Implement the method to create objects of the agents based on the data from the agents initialization file. Also define additional required attributes.
        agents = [Agent(agents=self, agentId=agents_initialization_data['agents'][0]['agentId'], agent_state='IDLE',
                        agent_order_topic=agents_initialization_data['agents'][0]['orderTopic'],
                        agent_state_topic=agents_initialization_data['agents'][0]['stateTopic'],
                        logging=self.logging)]
        return agents


class Agent:
    """
    Class representing a digital twin of an agent.
    """
    def __init__(self, agents, agentId, agent_state_topic, agent_order_topic, agent_state, logging) -> None:
        """
        Initialize the digital twin agent object.

        :param agents: The agents object containing the digital twin agents.
        :param agentId: The ID of the agent.
        :param agent_state: The state of the agent.
        :param agent_order_topic: The MQTT topic for the order messages of the agent.
        :param agent_state_topic: The MQTT topic for the state messages of the agent.
        :param logging: Logging object.
        """
        # TODO: Define required attributes of the agent to describe its state and behavior.

        self.agents = agents
        self.agentId = agentId
        self.agent_state = agent_state
        self.agvPosition = {}
        self.safetyState = {}
        self.loaded = False
        self.state_topic = agent_state_topic
        self.order_topic = agent_order_topic
        self.logging = logging
        self.mqtt_subscriber_state = MQTTSubscriber(config_data=self.agents.config_data, logging=self.logging, on_message=self.state_callback,
                                                    channel=self.state_topic, client_id=f'state_subscriber_agent_{self.agentId}')
        self.order_interface = OrderInterface(config_data=self.agents.config_data, logging=logging,
                                              order_topic=self.order_topic, agentId=self.agentId)
        self.moved_distance = 0
    
    def state_callback(self, client, userdata, msg) -> None:
        """
        Callback function for the MQTT message.

        :param client: MQTT client.
        :param userdata: User data.
        :param msg: Message.
        """
        self.logging.info(f"Client {self.mqtt_subscriber_state.client_id} received message from topic `{msg.topic}`.")  # `{msg.payload.decode()}`

        # TODO: Read out the state message automatically and use it to update the agent state.

        # TODO: Calculate the moved distance and log it to evaluate the path of the agent.
        # self.logging.info(f"Agent {self.agentId} has moved {round(self.moved_distance, 4)} meters.")
