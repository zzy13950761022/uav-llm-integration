import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DeadmanNode(Node):
    def __init__(self):
        super().__init__('deadman_node')
        # Subscribe to LLM commands
        self.subscription = self.create_subscription(
            String,
            '/llm_cmd',
            self.listener_callback,
            10
        )
        # Publisher to robot command topic
        self.publisher = self.create_publisher(String, '/cmd_vel', 10)

    def listener_callback(self, msg: String):
        self.get_logger().info(f"Safety Node received: {msg.data}")
        # For now, just pass the command through
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeadmanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()