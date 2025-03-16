import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import threading
import time

class DepthAICameraNode(Node):
    def __init__(self):
        super().__init__('custom_camera_node')
        # Create a publisher for the camera images
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        self.bridge = CvBridge()
        
        # Build the DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Create a ColorCamera node and an XLinkOut node in the pipeline
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRgb.setStreamName("rgb")

        # Configure camera properties
        self.camRgb.setPreviewSize(300, 300)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Link the preview output to the XLinkOut input
        self.camRgb.preview.link(self.xoutRgb.input)

        # Start the DepthAI loop in a separate thread
        self.depthai_thread = threading.Thread(target=self.depthai_loop, daemon=True)
        self.depthai_thread.start()

    def depthai_loop(self):
        # Adding a short delay to ensure the device is fully booted.
        time.sleep(2)
        try:
            with dai.Device(self.pipeline) as device:
                self.get_logger().info('DepthAI device connected.')
                self.get_logger().info('Connected cameras: ' + str(device.getConnectedCameraFeatures()))
                self.get_logger().info('USB speed: ' + device.getUsbSpeed().name)
                if device.getBootloaderVersion() is not None:
                    self.get_logger().info('Bootloader version: ' + str(device.getBootloaderVersion()))
                self.get_logger().info('Device name: ' + device.getDeviceName() +
                                         ' Product name: ' + device.getProductName())

                # Get the output queue for the rgb stream
                qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

                while rclpy.ok():
                    inRgb = qRgb.get()  # blocking call; waits for new frame
                    frame = inRgb.getCvFrame()  # Get the frame as an OpenCV image
                    try:
                        # Convert the OpenCV image (RGB) to a ROS Image message
                        msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
                        self.publisher_.publish(msg)
                    except Exception as e:
                        self.get_logger().error("Error converting or publishing image: " + str(e))
        except Exception as e:
            self.get_logger().error("Error in DepthAI loop: " + str(e))

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = DepthAICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()