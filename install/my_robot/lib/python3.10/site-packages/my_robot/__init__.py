import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.publisher_ = self.create_publisher(String, '/robot_commands', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.prev_command = "STOP"
        self.last_command_time = time.time()
        self.mode = "STOP"
        self.timer = self.create_timer(0.1, self.publish_command)

    def lidar_callback(self, scan):
        # (salin logika lama dari lidar_callback di ROS 1)
        # gunakan self.publisher_.publish(msg) nanti di timer
        pass

    def publish_command(self):
        msg = String()
        msg.data = self.prev_command
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
