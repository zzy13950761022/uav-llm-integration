# llm/llm_node.py
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        # Subscribe to the text input topic.
        self.text_sub = self.create_subscription(String, '/text_in', self.text_callback, 10)
        # Publisher for LLM command responses (to be forwarded by a safety node)
        self.cmd_pub = self.create_publisher(String, '/llm_cmd', 10)
        # API parameters
        self.api_key = os.environ.get('LLM_API_KEY')
        self.model = 'gpt-4'
        # Configurable API ping interval (in seconds)
        self.api_interval = 10.0
        self.last_api_time = 0.0
        self.latest_text = ""
        # Create a timer that checks periodically (every second)
        self.api_timer = self.create_timer(1.0, self.timer_callback)

    def text_callback(self, msg: String):
        # Update the latest text received.
        self.latest_text = msg.data
        self.get_logger().info(f"Received text input: {self.latest_text}")

    def timer_callback(self):
        # Get current time (in seconds)
        current_time = self.get_clock().now().nanoseconds / 1e9
        # If sufficient time has passed since the last successful API call and there is input:
        if self.latest_text and (current_time - self.last_api_time >= self.api_interval):
            self.get_logger().info("Attempting to ping LLM API...")
            success = self.call_api(self.latest_text)
            if success:
                # On a successful call, reset the timer.
                self.last_api_time = current_time
            else:
                # On failure, log and try again immediately (timer will fire in 1 second).
                self.get_logger().warn("LLM API call failed; will try again immediately.")

    def call_api(self, user_text: str) -> bool:
        url = "https://api.openai.com/v1/chat/completions"
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        # Construct a prompt using only the user text input.
        prompt = f"User command: {user_text}\nRespond with a JSON object in the following format: " \
                 "{\"linear\": <value in m/s>, \"angular\": <value in rad/s>}."
        data = {
            "model": self.model,
            "messages": [{"role": "user", "content": prompt}],
            "temperature": 0.7
        }
        try:
            response = requests.post(url, headers=headers, json=data)
            response.raise_for_status()
            result = response.json()
            response_text = result['choices'][0]['message']['content']
            self.get_logger().info(f"LLM API response: {response_text}")
            # Publish the response to /llm_cmd (safety node will later filter before going to /cmd_vel)
            msg = String()
            msg.data = response_text
            self.cmd_pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"LLM API call failed: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
