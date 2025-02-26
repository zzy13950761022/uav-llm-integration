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
        self.get_logger().info('Scanning available input devices:')
        for dev in devices:
            self.get_logger().info(f'Found device: {dev.name} at {dev.path}')
            if 'wireless controller' in dev.name.lower():
                self.device = dev
                self.get_logger().info(f'Selected device: {dev.name} at {dev.path}')
                break

        if self.device is None:
            self.get_logger().error('No joystick device found!')
            return

        # Dictionary to hold the current button and D-pad states.
        self.buttons = {}  # Stores keypresses
        self.axes = {}     # Stores D-pad inputs

        # Start a thread to read joystick events continuously.
        self.joy_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.joy_thread.start()

    def read_loop(self):
        try:
            for event in self.device.read_loop():
                if event.type == ecodes.EV_KEY:
                    # Handle regular button presses
                    key_name = ecodes.KEY.get(event.code, f'KEY_{event.code}')
                    self.buttons[key_name] = event.value  # 1 = Pressed, 0 = Released
                    # self.get_logger().info(f'Detected button: {key_name} = {event.value}')

                elif event.type == ecodes.EV_ABS:
                    # Handle D-pad (Hat Switch)
                    if event.code == ecodes.ABS_HAT0X:
                        self.axes['ABS_HAT0X'] = event.value  # Left (-1), Neutral (0), Right (1)
                    if event.code == ecodes.ABS_HAT0Y:
                        self.axes['ABS_HAT0Y'] = event.value  # Up (-1), Neutral (0), Down (1)
                    # self.get_logger().info(f'Detected D-pad: ABS_HAT0X={self.axes.get('ABS_HAT0X', 0)}, ABS_HAT0Y={self.axes.get('ABS_HAT0Y', 0)}')

                # Publish the current state (buttons + D-pad)
                state = {'buttons': self.buttons, 'axes': self.axes}
                msg = String()
                msg.data = json.dumps(state)
                self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error reading joystick: {e}')
    
    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
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