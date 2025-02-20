import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DeadmanNode(Node):
    def __init__(self):
        super().__init__('deadman_node')
        self.subscription = self.create_subscription(
            Twist,
            '/llm_cmd',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, msg: Twist):
        self.get_logger().info(f"Deadman Node received: linear={msg.linear.x}, angular={msg.angular.z}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeadmanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()