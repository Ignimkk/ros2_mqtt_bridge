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
        client.subscribe("mqtt/joint_trajectory")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            traj_msg = JointTrajectory()
            traj_msg.joint_names = [
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
            ]

            for pt in data.get("points", []):
                point = JointTrajectoryPoint()
                
                # pos_val 추출 후, 리스트가 아니면 리스트로 변환
                pos_val = pt.get("positions", [])
                if not isinstance(pos_val, list):
                    # 만약 pos_val이 int 또는 float라면 리스트로 감싸줌
                    if isinstance(pos_val, (int, float)):
                        pos_val = [pos_val]
                    else:
                        # 그 외 타입은 JSON 파싱 등으로 재검증
                        try:
                            pos_val = json.loads(pos_val)
                        except Exception as e:
                            self.get_logger().error(f"Error parsing positions value: {e}")
                            pos_val = []

                # 이제 pos_val이 리스트임이 보장됨
                point.positions = [float(x) for x in pos_val]


                # 기본값 처리 (velocities, accelerations, time_from_start)
                point.velocities = [0.5] * len(point.positions)
                point.accelerations = [0.1] * len(point.positions)
                point.time_from_start = Duration(sec=2, nanosec=0)

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
