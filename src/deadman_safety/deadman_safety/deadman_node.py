import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json

class DeadmanNode(Node):
    def __init__(self):
        super().__init__('deadman_node')
        # Publisher to output final command to UAV
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers:
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)
        self.custom_joy_sub = self.create_subscription(String, '/custom_joy_cmd', self.joy_callback, 10)
        self.llm_sub = self.create_subscription(Twist, '/llm_cmd', self.llm_callback, 10)
        
        # Safety state (from LiDAR)
        self.too_close = False
        self.safety_distance = 0.5  # meters
        
        # Latest messages from custom joy node and LLM
        self.joy_state = {}   # Will store parsed JSON from custom joy node.
        self.latest_llm_cmd = Twist()  # Fallback command.
        
        # **State tracking for reduced log spam**
        self.previous_deadman_state = None
        self.previous_command_active = None
        self.previous_safety_state = None
        
        # Timer: publishes final command at 10 Hz.
        self.timer = self.create_timer(0.1, self.publish_command)

    def lidar_callback(self, msg: LaserScan):
        try:
            min_distance = min(msg.ranges)
            self.too_close = min_distance < self.safety_distance
            
            # **Log only if safety state changes**
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
        final_cmd = Twist()  # Default command: stop.

        # Safety override: if obstacle is too close, always stop.
        if self.too_close:
            self.cmd_pub.publish(final_cmd)
            return

        # Check deadman switch using custom joy node JSON.
        # For the PS4 controller, L1 and R1 are detected as KEY_310 and KEY_311.
        buttons = self.joy_state.get("buttons", {})
        deadman_pressed = (buttons.get("KEY_310", 0) == 1 and buttons.get("KEY_311", 0) == 1)

        # **Log only if deadman state changes**
        if deadman_pressed != self.previous_deadman_state:
            if deadman_pressed:
                self.get_logger().info("Deadman switch engaged.")
            else:
                self.get_logger().info("Deadman switch not engaged. Stopping UAV.")
            self.previous_deadman_state = deadman_pressed  # Update state tracking

        if not deadman_pressed:
            self.cmd_pub.publish(final_cmd)
            return

        # If deadman is engaged, read axis values.
        axes = self.joy_state.get("axes", {})
        # Adjust these axis names if needed. Here we assume:
        #   - ABS_Y controls forward/backward (invert if necessary)
        #   - ABS_X controls rotation.
        scaling_factor_linear = 2.0    # Increase for more responsiveness.
        scaling_factor_angular = 1.5   # Increase for more sensitivity.

        linear_input = axes.get("ABS_Y", 0.0)
        angular_input = axes.get("ABS_X", 0.0)

        # Compute joystick command from axis values.
        computed_cmd = Twist()
        computed_cmd.linear.x = -linear_input * scaling_factor_linear  # Invert as needed.
        computed_cmd.angular.z = angular_input * scaling_factor_angular

        # **Check if joystick is active**
        command_active = abs(computed_cmd.linear.x) > 0.01 or abs(computed_cmd.angular.z) > 0.01

        # **Log only if movement state changes**
        if command_active != self.previous_command_active:
            if command_active:
                self.get_logger().info("Publishing UAV command from joystick input.")
            else:
                self.get_logger().info("No joystick input detected; using LLM command.")
            self.previous_command_active = command_active  # Update state tracking

        # If joystick axis command is essentially zero, fallback to LLM command.
        if command_active:
            final_cmd = computed_cmd
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