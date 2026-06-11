import json
import os
from jsonschema import validate, ValidationError, Draft7Validator
from paho.mqtt import client as mqtt_client

_SCHEMAS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "json_schemas")

class MQTTPublisher:
    # Class-level cache for schemas and validators
    _schema_cache = {}
    _validator_cache = {}
    
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
        
        # Pre-load and cache validators for this topic
        self._ensure_validator_cached(self.topic.split('/')[-1])
        
        self.client.connect(self.broker, self.port)
        self.client.loop_start()
    
    @classmethod
    def _load_schema(cls, schema_path):
        """
        Load and cache a JSON schema.
        
        :param schema_path: Path to the schema file.
        :return: The loaded schema.
        """
        if schema_path not in cls._schema_cache:
            with open(schema_path, "r", encoding="utf-8") as schema_file:
                cls._schema_cache[schema_path] = json.load(schema_file)
        return cls._schema_cache[schema_path]
    
    @classmethod
    def _get_validator(cls, schema_path):
        """
        Get or create a cached validator for a schema.
        
        :param schema_path: Path to the schema file.
        :return: A Draft7Validator instance.
        """
        if schema_path not in cls._validator_cache:
            schema = cls._load_schema(schema_path)
            cls._validator_cache[schema_path] = Draft7Validator(schema)
        return cls._validator_cache[schema_path]
    
    def _ensure_validator_cached(self, topic):
        """
        Pre-load validator for the given topic.
        
        :param topic: The topic name.
        """
        schema_map = {
            'order': os.path.join(_SCHEMAS_DIR, "order.schema"),
            'state': os.path.join(_SCHEMAS_DIR, "state.schema"),
        }
        
        if topic in schema_map:
            self._get_validator(schema_map[topic])

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
            
            # Validate the message based on the topic using cached validators
            schema_map = {
                'order': (os.path.join(_SCHEMAS_DIR, "order.schema"), 'Order'),
                'state': (os.path.join(_SCHEMAS_DIR, "state.schema"), 'State'),
            }
            
            if topic in schema_map:
                schema_path, topic_name = schema_map[topic]
                validator = self._get_validator(schema_path)
                self.validate_json_cached(message, validator, topic=topic_name)
            elif topic == 'tasks':
                # No validation for tasks message, as it is a custom message.
                pass
            else:
                self.logging.error(f"Topic {self.topic} is not supported.")
                return
            
            # Publish the message.
            result = self.client.publish(self.topic, json.dumps(message), qos=qos)
            status = result[0]

            # Log the status of the message.
            if status == 0:
                if topic == 'tasks':
                    self.logging.debug(f"Client {self.client_id} send message `{message}` to topic `{self.topic}`.")
                else:
                    self.logging.info(f"Client {self.client_id} send message `{message}` to topic `{self.topic}`.")  # `{message}`
            else:
                self.logging.error(f"Client {self.client_id} failed to send message to topic {self.topic}.")
        else:
            raise RuntimeError("Publish was called, before client was initialized.")

    def validate_json_cached(self, message, validator, topic) -> None:
        """
        Validate the message JSON using a cached validator.

        :param message: The message to validate.
        :param validator: The cached Draft7Validator instance.
        :param topic: The topic of the message.
        :raises ValidationError: If the message data does not conform to the schema.
        """
        try:
            validator.validate(message)
            self.logging.info(f"{topic} message is valid.")
        except ValidationError as e:
            self.logging.error(f"{topic} message validation failed: {e.message}")
            raise
    
    def validate_json(self, message, schema, topic) -> None:
        """
        Validate the message json based on the schema (legacy method for compatibility).

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
