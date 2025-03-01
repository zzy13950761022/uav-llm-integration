import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
from transformers import BlipProcessor, BlipForConditionalGeneration

class CameraEchoNode(Node):
    def __init__(self):
        super().__init__('camera_caption_node')
        self.publisher_ = self.create_publisher(String, '/camera_caption', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.window_name = "Camera View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        self.processor = None
        self.model = None
        self._model_lock = threading.Lock()
        self.current_caption = None  # Track the last caption

        # Eagerly load the model at startup
        self.load_model()

    def load_model(self):
        with self._model_lock:
            if self.model is None or self.processor is None:
                try:
                    self.get_logger().info("Loading BLIP model for image captioning...")
                    start_time = time.time()
                    self.processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
                    self.model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base")
                    elapsed = time.time() - start_time
                    self.get_logger().info(f"BLIP model loaded successfully in {elapsed:.2f} seconds.")
                except Exception as e:
                    self.get_logger().error(f"Failed to load BLIP model: {e}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Display the image (BGR is fine for OpenCV display)
        cv2.imshow(self.window_name, cv_image)
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("ESC pressed, shutting down Camera Echo Node.")
            self.on_shutdown()

        # Convert the image from BGR to RGB for correct color processing
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Lazy-load the captioning model if needed.
        if self.model is None or self.processor is None:
            self.load_model()
        if self.model is None or self.processor is None:
            self.get_logger().error("Captioning model not available; skipping caption generation.")
            return

        try:
            # Generate caption using the BLIP model on the RGB image
            inputs = self.processor(cv_image_rgb, return_tensors="pt")
            out = self.model.generate(**inputs, max_length=20)
            caption = self.processor.decode(out[0], skip_special_tokens=True)

            # Log only if the caption has changed
            if caption != self.current_caption:
                self.current_caption = caption
                self.get_logger().info(f"Caption updated: {caption}")
        except Exception as e:
            self.get_logger().error(f"Error generating caption: {e}")

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