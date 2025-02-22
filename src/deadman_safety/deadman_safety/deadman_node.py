import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json

class DeadmanNode(Node):
    def __init__(self):
        super().__init__('deadman_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)
        self.custom_joy_sub = self.create_subscription(String, '/custom_joy_cmd', self.joy_callback, 10)
        self.llm_sub = self.create_subscription(Twist, '/llm_cmd', self.llm_callback, 10)
        
        self.too_close = False
        self.safety_distance = 0.5  # meters
        
        self.joy_state = {}          # Will store JSON parsed from custom joy node (only buttons)
        self.latest_llm_cmd = Twist()  # Fallback command
        
        # State tracking for logging changes once.
        self.previous_deadman_state = None
        self.previous_command_active = None
        self.previous_safety_state = None
        
        # Timer to publish command at 10 Hz.
        self.timer = self.create_timer(0.1, self.publish_command)

    def lidar_callback(self, msg: LaserScan):
        try:
            min_distance = min(msg.ranges)
            self.too_close = min_distance < self.safety_distance
            if self.too_close != self.previous_safety_state:
                if self.too_close:
                    self.get_logger().warn(f"Obstacle detected at {min_distance:.2f}m! Stopping UAV.")
                else:
                    self.get_logger().info("Obstacle cleared, resuming movement.")
                self.previous_safety_state = self.too_close
        except Exception as e:
            self.get_logger().error(f"Lidar error: {e}")

    def joy_callback(self, msg: String):
        try:
            self.joy_state = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"JSON parse error in joy_callback: {e}")

    def llm_callback(self, msg: Twist):
        self.latest_llm_cmd = msg

    def publish_command(self):
        final_cmd = Twist()  # Default command: stop

        # 1. Safety override: if an obstacle is too close, always stop.
        if self.too_close:
            self.cmd_pub.publish(final_cmd)
            return

        # 2. Check deadman switch using custom joy node JSON.
        # For the PS4 controller, we expect L1 (KEY_310) and R1 (KEY_311) to be the deadman.
        buttons = self.joy_state.get("buttons", {})
        deadman_pressed = (buttons.get("KEY_310", 0) == 1 and buttons.get("KEY_311", 0) == 1)

        if deadman_pressed != self.previous_deadman_state:
            if deadman_pressed:
                self.get_logger().info("Deadman switch engaged.")
            else:
                self.get_logger().info("Deadman switch not engaged. Stopping UAV.")
            self.previous_deadman_state = deadman_pressed

        if not deadman_pressed:
            self.cmd_pub.publish(final_cmd)
            return

        # 3. Determine directional command from buttons.
        # Mapping:
        #   - KEY_307: forward → +0.5 m/s
        #   - KEY_304: reverse → -0.5 m/s
        #   - KEY_308: left    → +0.5 rad/s (turn left)
        #   - KEY_305: right   → -0.5 rad/s (turn right)
        FORWARD_SPEED = 0.5
        REVERSE_SPEED = -0.5
        TURN_LEFT_SPEED = 0.5
        TURN_RIGHT_SPEED = -0.5

        linear_speed = 0.0
        angular_speed = 0.0

        if buttons.get("KEY_307", 0) == 1:
            linear_speed += FORWARD_SPEED
        if buttons.get("KEY_304", 0) == 1:
            linear_speed += REVERSE_SPEED
        if buttons.get("KEY_308", 0) == 1:
            angular_speed += TURN_LEFT_SPEED
        if buttons.get("KEY_305", 0) == 1:
            angular_speed += TURN_RIGHT_SPEED

        # 4. If any directional button is pressed, use that command.
        command_active = (abs(linear_speed) > 0.01 or abs(angular_speed) > 0.01)

        if command_active != self.previous_command_active:
            if command_active:
                self.get_logger().info("Publishing UAV command from joystick buttons.")
            else:
                self.get_logger().info("No joystick button input; using LLM command.")
            self.previous_command_active = command_active

        if command_active:
            final_cmd.linear.x = linear_speed
            final_cmd.angular.z = angular_speed
        else:
            final_cmd = self.latest_llm_cmd

        self.cmd_pub.publish(final_cmd)

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f"Shutting down {self.get_name()}...")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DeadmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()