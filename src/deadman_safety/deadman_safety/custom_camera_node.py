import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import time

class CustomCameraNode(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        self.bridge = CvBridge()
        # Open the default webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Webcam could not be opened!")
            return
        # Start a separate thread to continuously capture frames
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()

    def capture_loop(self):
        '''
        Continuously capture frames from the webcam and publish them
        '''
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warning("No frame received from webcam")
                time.sleep(0.1)
                continue
            try:
                # Convert the OpenCV frame (BGR) to a ROS Image message
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Error converting/publishing frame: {e}")
            # Aim for ~30 fps
            time.sleep(1/30)

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = CustomCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()