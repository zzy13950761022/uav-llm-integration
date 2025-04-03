import os
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
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.threshold = int(os.getenv('AREA_THRESHOLD'))
        self.bridge = CvBridge()
        self.window_name = 'Masked View'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.processor = None
        self.model = None
        self._model_lock = threading.Lock()
        self.current_caption = None  # Track the last caption
        # Eagerly load the captioning model at startup.
        self.load_model()

    def load_model(self):
        """
        Load the BLIP model for image captioning
        This method is thread-safe and should be called only once
        """
        with self._model_lock:
            if self.model is None or self.processor is None:
                try:
                    self.get_logger().info('Loading BLIP model for image captioning...')
                    start_time = time.time()
                    self.processor = BlipProcessor.from_pretrained('Salesforce/blip-image-captioning-base')
                    self.model = BlipForConditionalGeneration.from_pretrained('Salesforce/blip-image-captioning-base')
                    elapsed = time.time() - start_time
                    self.get_logger().info(f'BLIP model loaded successfully in {elapsed:.2f} seconds')
                except Exception as e:
                    self.get_logger().error(f'Failed to load BLIP model: {e}')

    def detect_objects(self, image):
        """
        Detect colored blobs using HSV thresholding and perform basic shape recognition
        Returns:
          - objects: list of detected objects with label, shape, and bounding box
          - raw_masked_image: image for captioning, produced by alpha-blending color overlays onto the original image
          - display_masked_image: version with flat color fills and drawn bounding boxes/labels for visualization
        """
        # Convert image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        objects = []
        # Start with the original image for raw masking (to preserve textures and original colors)
        raw_masked_image = image.copy()
        # For display purposes, we use a white background initially.
        display_masked_image = np.ones_like(image) * 255
        # Define HSV color ranges and their corresponding BGR values.
        color_ranges = {
            'red': [((0, 100, 100), (10, 255, 255)), ((160, 100, 100), (180, 255, 255))],
            'blue': [((100, 150, 0), (140, 255, 255))],
            'yellow': [((20, 100, 100), (30, 255, 255))],
            'purple': [((130, 50, 50), (160, 255, 255))],
        }
        # These colors will be used for the display image (flat fill) and for blending in the raw image.
        color_bgr = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'purple': (128, 0, 128),
        }
        area_threshold = self.threshold
        # Process each color range.
        for color, ranges in color_ranges.items():
            combined_mask = None
            for lower, upper in ranges:
                lower_np = np.array(lower, dtype='uint8')
                upper_np = np.array(upper, dtype='uint8')
                current_mask = cv2.inRange(hsv, lower_np, upper_np)
                if combined_mask is None:
                    combined_mask = current_mask
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, current_mask)
            # Apply morphological operations to clean up the mask.
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            # --- Update raw_masked_image using alpha blending ---
            # Create an overlay image filled with the target color
            overlay = np.full(image.shape, color_bgr[color], dtype=np.uint8)
            alpha = 0.5  # blending factor
            # Blend the overlay with the raw image only where the mask is set
            # First, create a blended version for the entire image
            blended = cv2.addWeighted(raw_masked_image, 1 - alpha, overlay, alpha, 0)
            # Then, update only the masked regions
            raw_masked_image[mask != 0] = blended[mask != 0]
            # --- Update display_masked_image with flat color fills ---
            display_masked_image[mask != 0] = color_bgr[color]
            # Find contours for object detection
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > area_threshold]
            if valid_contours:
                merged_contour = np.concatenate(valid_contours)
                x, y, w, h = cv2.boundingRect(merged_contour)
                # Approximate the contour to recognize basic shapes
                peri = cv2.arcLength(merged_contour, True)
                approx = cv2.approxPolyDP(merged_contour, 0.04 * peri, True)
                shape = 'unidentified'
                if len(approx) == 3:
                    shape = 'triangle'
                elif len(approx) == 4:
                    shape = 'rectangle'
                elif len(approx) == 5:
                    shape = 'pentagon'
                elif len(approx) > 5:
                    shape = 'circle'
                objects.append({'label': color, 'shape': shape, 'bbox': (x, y, w, h)})
        # Now create a display version that shows bounding boxes and labels
        for obj in objects:
            x, y, w, h = obj['bbox']
            cv2.rectangle(display_masked_image, (x, y), (x+w, y+h), (0, 0, 0), 2)
            label_text = f"{obj['label']} {obj['shape']}"
            cv2.putText(display_masked_image, label_text, (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        return objects, raw_masked_image, display_masked_image

    def analyze_positions(self, objects, image_width):
        """
        For each detected object, determine its horizontal position: left, center, or right
        """
        positions = {}
        for obj in objects:
            x, y, w, h = obj['bbox']
            obj_center = x + w / 2
            if obj_center < image_width / 3:
                pos = 'left'
            elif obj_center > 2 * image_width / 3:
                pos = 'right'
            else:
                pos = 'center'
            positions[f"{obj['label']} {obj['shape']}"] = pos
        return positions

    def image_callback(self, msg):
        """
        Callback for camera image messages
        """
        try:
            # Convert ROS Image message to OpenCV image (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        # Run object detection to get objects and both versions of the masked image
        objects, raw_masked_image, display_masked_image = self.detect_objects(cv_image)
        positions = self.analyze_positions(objects, cv_image.shape[1])
        # Display the image with bounding boxes and labels
        cv2.imshow(self.window_name, display_masked_image)
        cv2.waitKey(1)
        # Use the raw masked image (with blended colors) for captioning
        masked_image_rgb = cv2.cvtColor(raw_masked_image, cv2.COLOR_BGR2RGB)
        # Ensure the captioning model is loaded
        if self.model is None or self.processor is None:
            self.load_model()
        if self.model is None or self.processor is None:
            self.get_logger().error('Captioning model not available; skipping caption generation.')
            return
        try:
            # Generate caption using the BLIP model on the raw masked image
            inputs = self.processor(masked_image_rgb, return_tensors='pt')
            out = self.model.generate(**inputs, max_length=20)
            caption = self.processor.decode(out[0], skip_special_tokens=True)
            # Append positional and shape information
            if positions:
                pos_descriptions = ', '.join([f'{desc} at the {pos}' for desc, pos in positions.items()])
                caption += f' ({pos_descriptions})'
            # Log and publish if the caption has changed
            if caption != self.current_caption:
                self.current_caption = caption
                self.get_logger().info(f'Caption updated: {caption}')
                self.publisher_.publish(String(data=caption))
        except Exception as e:
            self.get_logger().error(f'Error generating caption: {e}')

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
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