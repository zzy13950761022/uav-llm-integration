import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraEchoNode(Node):
    def __init__(self):
        super().__init__('camera_echo_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.window_name = "Camera View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        cv2.imshow(self.window_name, cv_image)
        # Check if ESC key is pressed (ASCII code 27)
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("ESC pressed, shutting down Camera Echo Node.")
            self.on_shutdown()

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f"Shutting down {self.get_name()}...")
        self.destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraEchoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()