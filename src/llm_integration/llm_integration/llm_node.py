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
        # Subscribe to user text input and image caption topics.
        self.text_sub = self.create_subscription(String, '/text_in', self.text_callback, 10)
        self.caption_sub = self.create_subscription(String, '/camera_caption', self.caption_callback, 10)
        # Publisher for LLM command responses.
        self.cmd_pub = self.create_publisher(Twist, '/llm_cmd', 10)
        # API parameters.
        self.api_key = os.environ.get('LLM_API_KEY')
        self.model = 'gpt-4'
        # Use a 5-second interval between successful API calls.
        self.api_interval = 5.0  
        self.last_api_time = 0.0
        self.latest_text = ''
        self.latest_caption = ''
        # Flag to indicate if a new caption has been received.
        self.caption_updated = False
        # Flag to pause API calls (set via LLM_STOP or blank command).
        self.llm_stopped = False
        # Flag to ensure the blank command message is only logged once.
        self.blank_logged = False
        # Stop timer for publishing zero velocities.
        self.stop_timer = None
        # Create a timer that checks every second.
        self.api_timer = self.create_timer(1.0, self.timer_callback)

    def text_callback(self, msg: String):
        '''Callback for text input messages.'''
        text = msg.data.strip()
        # Treat blank text as a pause command.
        if text == '':
            if not self.blank_logged:
                self.get_logger().info('User command is blank. Pausing LLM API calls.')
                self.blank_logged = True
            if not self.llm_stopped:
                self.llm_stopped = True
                stop_twist = Twist()  # zero velocities
                self.cmd_pub.publish(stop_twist)
                if self.api_timer is not None:
                    self.api_timer.cancel()
            return
        # Handle explicit stop command.
        if text == 'LLM_STOP':
            if not self.llm_stopped:
                self.get_logger().info('Received LLM_STOP command. Pausing API calls.')
                self.llm_stopped = True
                stop_twist = Twist()
                self.cmd_pub.publish(stop_twist)
                if self.api_timer is not None:
                    self.api_timer.cancel()
            return
        else:
            # Reset the blank flag and resume API calls if previously paused.
            self.blank_logged = False
            if self.llm_stopped:
                self.get_logger().info('Resuming LLM API calls.')
                self.llm_stopped = False
                self.api_timer = self.create_timer(1.0, self.timer_callback)
            self.latest_text = text
            # Force an immediate API call due to new text.
            self.get_logger().info('User command updated. Triggering LLM API call...')
            self.trigger_api_call(force=True)

    def caption_callback(self, msg: String):
        '''Callback for image caption messages.'''
        caption = msg.data.strip()
        if caption != self.latest_caption:
            self.latest_caption = caption
            self.caption_updated = True
            # Force an immediate API call due to updated caption.
            if self.latest_text.strip() and not self.blank_logged:
                self.get_logger().info('Caption updated. Triggering LLM API call...')
            self.trigger_api_call(force=True)

    def timer_callback(self):
        '''Callback for the API timer.'''
        if self.llm_stopped:
            return
        self.trigger_api_call(force=False)

    def trigger_api_call(self, force=False):
        '''Decide whether to trigger the API call immediately.'''
        if self.llm_stopped:
            return
        # Do not proceed if the user command is blank.
        if not self.latest_text.strip():
            return
        current_time = self.get_clock().now().nanoseconds / 1e9
        # If not forced, ensure that the API interval has elapsed.
        if not force and (current_time - self.last_api_time < self.api_interval):
            return

        # Construct the prompt including both the user command and image caption.
        prompt = (
            f'User command: {self.latest_text}\n'
            f'Image caption: {self.latest_caption}\n'
            f'Respond with a JSON object in the following format:\n'
            f'{{"linear": <value in m/s>, "angular": <value in rad/s>}}.\n'
            f'If desired object is stated in caption to be either left or right, turn first.\n'
            f'If desired object is stated in caption to be center, drive towards.\n'
            f'You may only change linear or angular, not both.\n'
            f'Maximum value for linear or angular is 0.5 m/s.\n'
            f'Negative angular values are clockwise.\n'
        )
        #self.get_logger().info('Triggering LLM API call with prompt:\n' + prompt)
        success = self.call_api(prompt)
        if success:
            self.last_api_time = current_time
            self.caption_updated = False
        else:
            self.get_logger().warn('LLM API call failed; will try again immediately.')

    def call_api(self, prompt: str) -> bool:
        '''Call the API with the composed prompt.'''
        url = 'https://api.openai.com/v1/chat/completions'
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}'
        }
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
                # Schedule a stop command after a set amount of seconds.
                self.schedule_stop()
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f'LLM API call failed: {e}')
            return False

    def schedule_stop(self):
        '''Schedule a one-shot timer to publish a stop command after a set amount of seconds.'''
        # Cancel any existing stop timer.
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        # Create a new timer that calls stop_callback after 2.5 seconds.
        self.stop_timer = self.create_timer(2.5, self.stop_callback)

    def stop_callback(self):
        '''Callback to publish zero velocities and cancel the stop timer.'''
        stop_twist = Twist()  # zero velocities
        self.cmd_pub.publish(stop_twist)
        self.get_logger().info('Awaiting new LLM command...')
        # Cancel the timer so it does not repeatedly trigger.
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None

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