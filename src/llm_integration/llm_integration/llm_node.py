import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import requests
import json
import base64
import os

class LLMController(Node):
    def __init__(self):
        super().__init__('llm_controller')
        self.bridge = CvBridge()

        # Subscribers: camera image and text input
        self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.create_subscription(String, '/text_input', self.text_callback, 10)

        # Publisher: movement commands to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters for LLM API
        self.api_key = os.environ.get('LLM_API_KEY')
        self.model = 'gpt-4'

        # Hold the latest inputs
        self.latest_text_input = "start"  # default command if nothing is provided
        self.latest_image = None

    def text_callback(self, msg: String):
        self.latest_text_input = msg.data
        self.get_logger().info(f"Received text input: {self.latest_text_input}")

    def image_callback(self, msg: Image):
        self.latest_image = msg
        self.get_logger().info("Received new camera image")
        self.process_inputs()

    def process_inputs(self):
        if self.latest_image is None:
            self.get_logger().warning("No image available yet!")
            return

        # Convert ROS image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        # Convert to a base64 string:
        retval, buffer = cv2.imencode('.jpg', cv_image)
        image_base64 = base64.b64encode(buffer).decode('utf-8')

        # Construct JSON payload for LLM API
        payload = {
            "user_command": self.latest_text_input,
            "image_description": image_base64
        }

        self.get_logger().info(f"Payload for LLM: {json.dumps(payload)}")
        response_text = self.call_chatgpt_api(payload)
        self.get_logger().info(f"LLM response: {response_text}")

        # Parse the JSON response from ChatGPT
        twist_cmd = self.parse_response(response_text)
        if twist_cmd:
            self.cmd_pub.publish(twist_cmd)
            self.get_logger().info("Published movement command.")
        else:
            self.get_logger().error("Failed to parse movement command from LLM response.")

    def call_chatgpt_api(self, payload: dict) -> str:
        url = "https://api.openai.com/v1/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        # Construct a message instructing LLM to respond in JSON:
        message_content = (
            f"User command: {payload['user_command']}\n"
            f"Camera view description: {payload['image_description']}\n"
            "Respond with a JSON object in the following format: "
            '{"linear": <linear_velocity in m/s>, "angular": <angular_velocity in rad/s>}.'
        )

        data = {
            "model": self.model,
            "messages": [{"role": "user", "content": message_content}],
            "temperature": 0.7
        }
        try:
            response = requests.post(url, headers=headers, json=data)
            response.raise_for_status()
            result = response.json()
            return result['choices'][0]['message']['content']
        except Exception as e:
            self.get_logger().error(f"LLM API call failed: {e}")
            return ""

    def parse_response(self, response_text: str):
        # Attempt to parse the response text as JSON.
        try:
            # If LLM returns additional text, might need to extract the JSON substring.
            # For now, assume it returns a clean JSON string.
            command = json.loads(response_text)
            twist = Twist()
            twist.linear.x = float(command.get("linear", 0.0))
            twist.angular.z = float(command.get("angular", 0.0))
            return twist
        except Exception as e:
            self.get_logger().error(f"Error parsing response JSON: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = LLMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
