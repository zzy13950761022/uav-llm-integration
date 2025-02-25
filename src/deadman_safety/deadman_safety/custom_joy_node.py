import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
from evdev import InputDevice, ecodes, list_devices

class CustomJoyNode(Node):
    def __init__(self):
        super().__init__('custom_joy_node')
        self.publisher_ = self.create_publisher(String, '/custom_joy_cmd', 10)

        # Look for the PS4 controller by name.
        devices = [InputDevice(path) for path in list_devices()]
        self.device = None
        self.get_logger().info("Scanning available input devices:")
        for dev in devices:
            self.get_logger().info(f"Found device: {dev.name} at {dev.path}")
            if "wireless controller" in dev.name.lower():
                self.device = dev
                self.get_logger().info(f"Selected device: {dev.name} at {dev.path}")
                break

        if self.device is None:
            self.get_logger().error("No joystick device found!")
            return

        # Dictionary to hold the current button state.
        self.buttons = {}

        # Start a thread to read joystick events continuously.
        self.joy_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.joy_thread.start()

    def read_loop(self):
        try:
            for event in self.device.read_loop():
                if event.type == ecodes.EV_KEY:
                    key_name = ecodes.KEY.get(event.code, f"KEY_{event.code}")
                    self.buttons[key_name] = event.value
                    # self.get_logger().info(f"Detected button: {key_name} = {event.value}")
                    # Publish the current button state as JSON.
                    state = {"buttons": self.buttons}
                    msg = String()
                    msg.data = json.dumps(state)
                    self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error reading joystick: {e}")
    
    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f"Shutting down {self.get_name()}...")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CustomJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()