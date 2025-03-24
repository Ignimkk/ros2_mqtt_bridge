import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import paho.mqtt.client as mqtt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MqttRosBridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros_bridge')

        qos_profile = QoSProfile(
            depth=5,  
            reliability=QoSReliabilityPolicy.RELIABLE,  
            history=QoSHistoryPolicy.KEEP_LAST  
        )
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            qos_profile
        )

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("localhost", 1883, 60)  

        self.get_logger().info("MQTT-ROS Bridge Node Started")

    def joint_states_callback(self, msg):
        joint_data = {
            "header": {
                "stamp": {
                    "sec": msg.header.stamp.sec,
                    "nanosec": msg.header.stamp.nanosec
                },
                "frame_id": msg.header.frame_id
            },
            "name": list(msg.name),
            "position": [round(pos, 2) for pos in msg.position],
            "velocity": [round(vel, 2) for vel in msg.velocity],  
            "effort": [round(eff, 2) for eff in msg.effort]
        }

        json_data = json.dumps(joint_data) 
        self.mqtt_client.publish("ros/joint_states", json_data, qos=0)  

        parsed_data = json.loads(json_data)
        self.get_logger().info(f"Published: {parsed_data['header']['stamp']['sec']}")


def main(args=None):
    rclpy.init(args=args)
    node = MqttRosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
