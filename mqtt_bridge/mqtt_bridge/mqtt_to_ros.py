# import rclpy
# from rclpy.node import Node
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# import json
# import paho.mqtt.client as mqtt

# class MqttToRos(Node):
#     def __init__(self):
#         super().__init__('mqtt_to_ros')

#         self.publisher = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

#         self.mqtt_client = mqtt.Client()
#         self.mqtt_client.on_connect = self.on_connect
#         self.mqtt_client.on_message = self.on_message

#         self.mqtt_client.connect("localhost", 1883, 60)  # Mosquitto 브로커 연결
#         self.mqtt_client.loop_start()

#         self.get_logger().info("MQTT to ROS Bridge Node Started")

#     def on_connect(self, client, userdata, flags, rc):
#         self.get_logger().info("Connected to MQTT Broker!")
#         client.subscribe("ros2/joint_trajectory")  # MQTT 구독

#     def on_message(self, client, userdata, msg):
#         try:
#             data = json.loads(msg.payload.decode("utf-8"))

#             joint_trajectory = JointTrajectory()
#             joint_trajectory.joint_names = data["joint_names"]

#             point = JointTrajectoryPoint()
#             point.positions = data["points"][0]["positions"]
#             point.velocities = data["points"][0]["velocities"]
#             point.accelerations = data["points"][0]["accelerations"]
#             point.time_from_start = Duration(sec=data["points"][0]["time_from_start"]["sec"],
#                                              nanosec=data["points"][0]["time_from_start"]["nanosec"])

#             joint_trajectory.points.append(point)

#             self.publisher.publish(joint_trajectory)
#             self.get_logger().info(f"Published to ROS2: {joint_trajectory}")

#         except Exception as e:
#             self.get_logger().error(f"Failed to process MQTT message: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = MqttToRos()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#################################
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json
import paho.mqtt.client as mqtt

class MqttToRos(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros')

        self.default_joint_names = [
            "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
            "wrist_2_joint", "wrist_3_joint", "shoulder_pan_joint"
        ]

        self.default_positions = [-1.57, 0.0, -1.57, 0.0, 0.0, 0.0]

        # self.publisher = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect("localhost", 1883, 60) 
        self.mqtt_client.loop_start()

        self.get_logger().info("MQTT to ROS Bridge Node Started")

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker!")
        client.subscribe("ros/joint_trajectory")  

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))

            joint_trajectory = JointTrajectory()
            
            joint_trajectory.joint_names = data.get("joint_names", self.default_joint_names)

            # JointTrajectoryPoint 생성
            point = JointTrajectoryPoint()
            point.positions = data["points"][0].get("positions", self.default_positions)
            time_from_start = data["points"][0].get("time_from_start", {"sec": 3, "nanosec": 0})
            point.time_from_start = Duration(sec=time_from_start["sec"], nanosec=time_from_start["nanosec"])

            # 메시지 퍼블리시
            joint_trajectory.points.append(point)
            self.publisher.publish(joint_trajectory)

            self.get_logger().info(f"Published to ROS2: {joint_trajectory}")

        except Exception as e:
            self.get_logger().error(f"Failed to process MQTT message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttToRos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
