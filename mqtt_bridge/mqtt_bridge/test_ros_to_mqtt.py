import rclpy
from rclpy.node import Node
import yaml
import json
import importlib
import paho.mqtt.client as mqtt
import os
from ament_index_python.packages import get_package_share_directory

def ros_msg_to_dict(msg):
    if isinstance(msg, (bool, int, float, str, type(None))):
        return msg
    elif isinstance(msg, (list, tuple)):
        return [ros_msg_to_dict(e) for e in msg]
    elif hasattr(msg, '__slots__'):
        return {
            field.lstrip('_'): ros_msg_to_dict(getattr(msg, field))
            for field in msg.__slots__
        }
    elif hasattr(msg, 'sec') and hasattr(msg, 'nanosec'):
        return {"sec": msg.sec, "nanosec": msg.nanosec}
    else:
        return str(msg)


class Ros2YamlSubscriber(Node):
    def __init__(self, yaml_file='ros_to_mqtt.yaml'):
        super().__init__('ros2_yaml_subscriber')

        pkg_share_dir = get_package_share_directory('mqtt_bridge')
        config_path = os.path.join(pkg_share_dir, 'config', yaml_file)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("localhost", 1883, 60)

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.subscribers = []
        for entry in config.get('topics', []):
            ros_topic = entry['ros_topic']
            msg_type = entry['msg_type']
            mqtt_topic = entry['mqtt_topic']
            qos = int(entry.get('qos', 5))

            msg_module_path, msg_class_name = self.parse_msg_type(msg_type)
            msg_module = importlib.import_module(msg_module_path)
            msg_class = getattr(msg_module, msg_class_name)

            callback = self.make_callback(mqtt_topic)
            subscriber = self.create_subscription(msg_class, ros_topic, callback, qos)
            self.subscribers.append(subscriber)
            self.get_logger().info(f'Subscribed to {ros_topic} ({msg_type}) → MQTT: {mqtt_topic}')

    def parse_msg_type(self, full_type):
        parts = full_type.split('/')
        return f"{parts[0]}.msg", parts[2]

    def make_callback(self, mqtt_topic):
        def callback(msg):
            msg_dict = ros_msg_to_dict(msg)
            json_msg = json.dumps(msg_dict)
            self.mqtt_client.publish(mqtt_topic, json_msg, qos=0)
            self.get_logger().info(f'Published to {mqtt_topic}')
        return callback

def main():
    rclpy.init()
    node = Ros2YamlSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
