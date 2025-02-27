import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests
import json
import re

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        # Subscribe to the text input topic.
        self.text_sub = self.create_subscription(String, '/text_in', self.text_callback, 10)
        # Publisher for LLM command responses (to be forwarded by a safety node)
        self.cmd_pub = self.create_publisher(Twist, '/llm_cmd', 10)
        # API parameters
        self.api_key = os.environ.get('LLM_API_KEY')
        self.model = 'gpt-4'
        # Configurable API ping interval (in seconds)
        self.api_interval = 10.0
        self.last_api_time = 0.0
        self.latest_text = ''
        # Flag to pause API calls if LLM_STOP command is received
        self.llm_stopped = False
        # Create a timer that checks periodically (every second)
        self.api_timer = self.create_timer(1.0, self.timer_callback)

    def text_callback(self, msg: String):
        '''Callback for text input messages.'''
        text = msg.data.strip()
        if text == 'LLM_STOP':
            if not self.llm_stopped:
                self.get_logger().info('Received LLM_STOP command. Sending stop message to API and pausing API calls.')
                self.llm_stopped = True
                # Publish a stop command (zero velocities) to /llm_cmd.
                stop_twist = Twist()
                self.cmd_pub.publish(stop_twist)
                # Cancel the API timer to prevent repeated "stop" logs.
                if self.api_timer is not None:
                    self.api_timer.cancel()
            return
        else:
            # If previously stopped, resume by re-creating the timer.
            if self.llm_stopped:
                self.get_logger().info('Resuming LLM API calls.')
                self.llm_stopped = False
                # Re-create the timer with the same period and callback.
                self.api_timer = self.create_timer(1.0, self.timer_callback)
            self.latest_text = text
            self.get_logger().info(f'Received text input: {self.latest_text}')

    def timer_callback(self):
        '''Callback for the API timer.'''
        # If the node is paused, do nothing.
        if self.llm_stopped:
            return
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.latest_text and (current_time - self.last_api_time >= self.api_interval):
            self.get_logger().info('Attempting to ping LLM API...')
            success = self.call_api(self.latest_text)
            if success:
                self.last_api_time = current_time
            else:
                self.get_logger().warn('LLM API call failed; will try again immediately.')

    def call_api(self, user_text: str) -> bool:
        '''Call the API with the user's text input.'''
        url = 'https://api.openai.com/v1/chat/completions'
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}'
        }
        prompt = (
            f'User command: {user_text}\n'
            f'Respond with a JSON object in the following format: '
            f'{{"linear": <value in m/s>, "angular": <value in rad/s>}}. '
            f'Maximum value for linear is 0.5 m/s.'
        )
        data = {
            'model': self.model,
            'messages': [{'role': 'user', 'content': prompt}],
            'temperature': 0.7
        }
        try:
            response = requests.post(url, headers=headers, json=data)
            response.raise_for_status()
            result = response.json()
            response_text = result['choices'][0]['message']['content']
            self.get_logger().info(f'LLM API response: {response_text}')
            twist_cmd = self.parse_response(response_text)
            if twist_cmd:
                self.cmd_pub.publish(twist_cmd)
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f'LLM API call failed: {e}')
            return False

    def parse_response(self, response_text: str):
        '''Parse the API response text into a Twist message.'''
        try:
            json_regex = re.compile(r'\{.*\}', re.DOTALL)
            match = json_regex.search(response_text)
            json_str = match.group(0) if match else response_text
            command = json.loads(json_str)
            twist = Twist()
            twist.linear.x = float(command.get('linear', 0.0))
            twist.angular.z = float(command.get('angular', 0.0))
            return twist
        except Exception as e:
            self.get_logger().error(f'Error parsing response JSON: {e}')
            return None

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()