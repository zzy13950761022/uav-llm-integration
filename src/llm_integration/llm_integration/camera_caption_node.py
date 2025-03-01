import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import numpy as np
from transformers import BlipProcessor, BlipForConditionalGeneration

class CameraCaptionNode(Node):
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

        # Eagerly load the captioning model at startup
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

    def detect_objects(self, image):
        """
        Detect colored blobs in the image using basic HSV thresholding.
        For each color, any contours with sufficient area are merged into a single bounding box.
        Returns a list of detected objects with labels and bounding boxes:
        [
            {"label": "red", "bbox": (x, y, w, h)},
            {"label": "blue", "bbox": (x, y, w, h)},
            ...
        ]
        """
        # Convert image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        objects = []
        
        # Define HSV color ranges.
        # Note: Red is defined in two parts due to the HSV hue wrap-around.
        color_ranges = {
            "red": [((0, 100, 100), (10, 255, 255)), ((160, 100, 100), (180, 255, 255))],
            "blue": [((100, 150, 0), (140, 255, 255))],
            "green": [((40, 70, 70), (80, 255, 255))],
            "yellow": [((20, 100, 100), (30, 255, 255))]
        }
        
        # Area threshold for filtering out small contours (adjust as needed)
        area_threshold = 500
        
        for color, ranges in color_ranges.items():
            combined_mask = None
            # Combine masks for all ranges of this color
            for lower, upper in ranges:
                lower_np = np.array(lower, dtype="uint8")
                upper_np = np.array(upper, dtype="uint8")
                current_mask = cv2.inRange(hsv, lower_np, upper_np)
                if combined_mask is None:
                    combined_mask = current_mask
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, current_mask)
            
            # Clean up the mask with morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Filter out small contours
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > area_threshold]
            
            if valid_contours:
                # Merge all valid contours into one bounding box
                x_min = float('inf')
                y_min = float('inf')
                x_max = 0
                y_max = 0
                for cnt in valid_contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                    x_min = min(x_min, x)
                    y_min = min(y_min, y)
                    x_max = max(x_max, x + w)
                    y_max = max(y_max, y + h)
                bbox = (int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min))
                objects.append({"label": color, "bbox": bbox})
        return objects

    def analyze_positions(self, objects, image_width):
        """
        Given a list of detected objects and the image width,
        determine each object's relative horizontal position: left, center, or right.
        """
        positions = {}
        for obj in objects:
            x, y, w, h = obj["bbox"]
            # Calculate the horizontal center of the bounding box
            obj_center = x + w / 2
            if obj_center < image_width / 3:
                pos = "left"
            elif obj_center > 2 * image_width / 3:
                pos = "right"
            else:
                pos = "center"
            positions[obj["label"]] = pos
        return positions

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Display the image
        cv2.imshow(self.window_name, cv_image)
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("ESC pressed, shutting down Camera Caption Node.")
            self.on_shutdown()

        # Convert the image from BGR to RGB for correct captioning
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Run object detection to get positional data
        objects = self.detect_objects(cv_image)
        positions = self.analyze_positions(objects, cv_image.shape[1])
        
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
            
            # If positional info is available, augment the caption
            if positions:
                pos_descriptions = ", ".join([f"{label} on the {pos}" for label, pos in positions.items()])
                caption += f" ({pos_descriptions})"
            
            # Log and publish only if the caption has changed
            if caption != self.current_caption:
                self.current_caption = caption
                self.get_logger().info(f"Caption updated: {caption}")
                self.publisher_.publish(String(data=caption))
        except Exception as e:
            self.get_logger().error(f"Error generating caption: {e}")

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f"Shutting down {self.get_name()}...")
        self.destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()