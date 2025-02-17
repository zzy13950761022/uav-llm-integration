import tkinter as tk
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class TextInNode(Node):
    def __init__(self):
        super().__init__('text_in_gui')
        self.publisher_ = self.create_publisher(String, '/text_in', 10)
        self.root = tk.Tk()
        self.root.title("UAV Command Input")
        self.entry = tk.Entry(self.root, width=50)
        self.entry.pack(padx=20, pady=10)
        self.button = tk.Button(self.root, text="Send Command", command=self.send_command)
        self.button.pack(padx=20, pady=10)
        # Set a protocol to handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def send_command(self):
        command = self.entry.get()
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: {command}")
        self.entry.delete(0, tk.END)

    def on_closing(self):
        self.root.quit()

    def spin(self):
        # Run Tkinter mainloop in a non-blocking way
        while rclpy.ok():
            self.root.update()
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = TextInNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()