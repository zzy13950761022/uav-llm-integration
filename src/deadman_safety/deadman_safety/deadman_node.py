import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class DeadmanNode(Node):
    def __init__(self):
        super().__init__('deadman_node')
        # Subscribe to LLM commands (Twist messages)
        self.llm_sub = self.create_subscription(
            Twist,
            '/llm_cmd',
            self.cmd_callback,
            10
        )
        # Subscribe to LiDAR sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )
        # Publisher to send final movement commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Define minimum safety distance (in meters)
        self.safety_distance = 0.5
        # Flag indicating whether an obstacle is too close
        self.too_close = False

        # Timer to continuously check and publish stop commands if needed (every 0.2 seconds)
        self.stop_timer = self.create_timer(0.2, self.publish_stop_if_necessary)

    def lidar_callback(self, msg: LaserScan):
        # Check the minimum range from the LiDAR scan data
        min_distance = min(msg.ranges)
        if min_distance < self.safety_distance:
            self.get_logger().warn(f"Obstacle detected at {min_distance:.2f}m! Stopping UAV.")
            self.too_close = True
        else:
            self.too_close = False

    def cmd_callback(self, msg: Twist):
        # If an obstacle is too close, ignore the LLM command and publish a stop command.
        if self.too_close:
            stop_msg = Twist()  # All velocities zero
            self.cmd_pub.publish(stop_msg)
            self.get_logger().info("LLM command overridden: Obstacle detected, stopping UAV!")
        else:
            self.cmd_pub.publish(msg)

    def publish_stop_if_necessary(self):
        # This timer callback continuously publishes a stop command if an obstacle is too close.
        if self.too_close:
            stop_msg = Twist()  # Zero velocities
            self.cmd_pub.publish(stop_msg)
            self.get_logger().info("Publishing periodic stop command due to obstacle.")

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f"Shutting down {self.get_name()}...")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DeadmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()