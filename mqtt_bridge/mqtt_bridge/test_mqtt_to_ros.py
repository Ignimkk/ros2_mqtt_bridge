import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Time, Duration
import paho.mqtt.client as mqtt
import json

class MqttToRos2Publisher(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros2_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect('localhost', 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe("ros/joint_trajectory")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            traj_msg = JointTrajectory()

            # traj_msg.header = Header()
            # traj_msg.header.stamp = self.get_clock().now().to_msg()

            traj_msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

            for pt in data.get("points", []):
                point = JointTrajectoryPoint()

                # positions (필수, float 변환)
                point.positions = [float(x) for x in pt.get("positions", [])]

                # velocities (옵션, 기본값 설정)
                velocities = pt.get("velocities", [0.5] * len(point.positions))
                point.velocities = [float(x) for x in velocities]

                # accelerations (옵션, 기본값 설정)
                accelerations = pt.get("accelerations", [0.1] * len(point.positions))
                point.accelerations = [float(x) for x in accelerations]

                # time_from_start (옵션)
                if "time_from_start" in pt:
                    duration = pt["time_from_start"]
                    point.time_from_start = Duration(sec=duration.get("sec", 0),
                                                    nanosec=duration.get("nanosec", 0))

                traj_msg.points.append(point)


            self.publisher_.publish(traj_msg)
            self.get_logger().info("Published JointTrajectory to ROS2")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttToRos2Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
