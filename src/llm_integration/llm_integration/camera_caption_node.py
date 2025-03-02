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
        self.window_name = 'Masked View'
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
                    self.get_logger().info('Loading BLIP model for image captioning...')
                    start_time = time.time()
                    self.processor = BlipProcessor.from_pretrained('Salesforce/blip-image-captioning-base')
                    self.model = BlipForConditionalGeneration.from_pretrained('Salesforce/blip-image-captioning-base')
                    elapsed = time.time() - start_time
                    self.get_logger().info(f'BLIP model loaded successfully in {elapsed:.2f} seconds.')
                except Exception as e:
                    self.get_logger().error(f'Failed to load BLIP model: {e}')

    def detect_objects(self, image):
        '''
        Detect colored blobs using HSV thresholding and perform basic shape recognition.
        Returns a list of detected objects with color, shape, and bounding box,
        along with a composite masked image showing the segmentation.
        '''
        # Convert image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        objects = []
        # Create a blank masked image (black background)
        masked_image = np.zeros_like(image)

        # Define HSV color ranges (red uses two ranges) and corresponding BGR values
        color_ranges = {
            'red': [((0, 100, 100), (10, 255, 255)), ((160, 100, 100), (180, 255, 255))],
            'blue': [((100, 150, 0), (140, 255, 255))],
            'green': [((40, 70, 70), (80, 255, 255))],
            'yellow': [((20, 100, 100), (30, 255, 255))]
        }
        color_bgr = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'yellow': (0, 255, 255)
        }
        # Minimum area to filter out noise
        area_threshold = 500

        for color, ranges in color_ranges.items():
            combined_mask = None
            # Combine masks for all ranges of this color
            for lower, upper in ranges:
                lower_np = np.array(lower, dtype='uint8')
                upper_np = np.array(upper, dtype='uint8')
                current_mask = cv2.inRange(hsv, lower_np, upper_np)
                if combined_mask is None:
                    combined_mask = current_mask
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, current_mask)
            
            # Apply morphological operations to clean up the mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Overlay the current mask onto the composite masked image
            masked_image[mask != 0] = color_bgr[color]
            
            # Find contours and filter based on area
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > area_threshold]
            
            if valid_contours:
                # Merge all valid contours into one
                merged_contour = np.concatenate(valid_contours)
                x, y, w, h = cv2.boundingRect(merged_contour)
                
                # Use contour approximation for basic shape recognition
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
                
                # Draw bounding box and label on the masked image
                cv2.rectangle(masked_image, (x, y), (x+w, y+h), (255, 255, 255), 2)
                label_text = f'{color} {shape}'
                cv2.putText(masked_image, label_text, (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return objects, masked_image

    def analyze_positions(self, objects, image_width):
        '''
        For each detected object, determine its horizontal position: left, center, or right.
        '''
        positions = {}
        for obj in objects:
            x, y, w, h = obj['bbox']
            # Calculate horizontal center of the bounding box
            obj_center = x + w / 2
            if obj_center < image_width / 3:
                pos = 'left'
            elif obj_center > 2 * image_width / 3:
                pos = 'right'
            else:
                pos = 'center'
            # Combine color and shape info for a richer description
            positions[f'{obj['label']} {obj['shape']}'] = pos
        return positions

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Run object detection to obtain positional and shape info plus a masked image
        objects, masked_image = self.detect_objects(cv_image)
        positions = self.analyze_positions(objects, cv_image.shape[1])
        
        # Display the masked image instead of the raw camera feed
        cv2.imshow(self.window_name, masked_image)
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info('ESC pressed, shutting down Camera Caption Node.')
            self.on_shutdown()

        # Convert image to RGB for captioning
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Ensure the captioning model is loaded
        if self.model is None or self.processor is None:
            self.load_model()
        if self.model is None or self.processor is None:
            self.get_logger().error('Captioning model not available; skipping caption generation.')
            return

        try:
            # Generate caption using the BLIP model on the RGB image
            inputs = self.processor(cv_image_rgb, return_tensors='pt')
            out = self.model.generate(**inputs, max_length=20)
            caption = self.processor.decode(out[0], skip_special_tokens=True)
            
            # Augment the caption with positional and shape information
            if positions:
                pos_descriptions = ', '.join([f'{desc} at the {pos}' for desc, pos in positions.items()])
                caption += f' ({pos_descriptions})'
            
            # Log and publish only if the caption has changed
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