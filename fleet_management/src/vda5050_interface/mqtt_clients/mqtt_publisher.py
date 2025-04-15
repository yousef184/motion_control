import json
from jsonschema import validate, ValidationError
from paho.mqtt import client as mqtt_client

class MQTTPublisher:
    def __init__(self, config_data, channel, client_id, logging) -> None:
        """
        Initialize the MQTT publisher.
        
        :param config_data: Data from the configuration file.
        :param channel: MQTT channel.
        :param client_id: MQTT client ID.
        :param logging: Logging object.
        """
        self.broker = config_data['mqtt_broker_ip']
        self.port = config_data['mqtt_broker_port']
        self.topic = channel
        self.client_id = client_id
        self.logging = logging
        self.client = mqtt_client.Client()
        self.client.on_connect = self.on_connect
        self.client.connect(self.broker, self.port)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc) -> None:
        """
        Callback function for the MQTT client connection.
        
        :param client: MQTT client.
        :param userdata: User data.
        :param flags: Flags.
        :param rc: Return code.
        """
        if rc == 0:
            self.logging.info(f"Connected client {self.client_id} to MQTT broker.")
        else:
            self.logging.error(f"Failed to connect client {self.client_id} to MQTT broker, return code {rc}.")

    def publish(self, message, qos:int) -> None:
        """
        Publish a message to the MQTT broker.

        :param message: Message to be published.
        :param qos: Quality of Service.
        """
        if self.client is not None:
            topic = self.topic.split('/')[-1]
            # Validate the message based on the topic.
            if topic == 'order':
                # Valdate the order message based on the order message schema from the VDA5050 standard.
                schema_path = "src/vda5050_interface/json_schemas/order.schema"
                with open(schema_path, "r") as schema_file:
                    schema = json.load(schema_file)
                self.validate_json(message, schema, topic='Order')

            elif topic == 'state':
                # Valdate the state message based on the state message schema from the VDA5050 standard.
                schema_path = "src/vda5050_interface/json_schemas/state.schema"
                with open(schema_path, "r", encoding="utf-8") as schema_file:
                    schema = json.load(schema_file)
                self.validate_json(message, schema, topic='State')
            
            else:
                self.logging.error(f"Topic {self.topic} is not supported.")
                return
            
            # Publish the message.
            result = self.client.publish(self.topic, json.dumps(message), qos=qos)
            status = result[0]

            # Log the status of the message.
            if status == 0:
                self.logging.info(f"Client {self.client_id} send message `{message}` to topic `{self.topic}`.")  # `{message}`
            else:
                self.logging.error(f"Client {self.client_id} failed to send message to topic {self.topic}.")
        else:
            raise RuntimeError("Publish was called, before client was initialized.")

    def validate_json(self, message, schema, topic) -> None:
        """
        Validate the message json based on the schema.

        :param message: The message.
        :param schema: The schema of the message.
        :param topic: The topic of the message.
        :raises ValidationError: If the message data does not conform to the schema.
        """
        try:
            validate(instance=message, schema=schema)
            self.logging.info(f"{topic} message is valid.")
        except ValidationError as e:
            self.logging.error(f"{topic} message validation failed: {e.message}")
            raise
