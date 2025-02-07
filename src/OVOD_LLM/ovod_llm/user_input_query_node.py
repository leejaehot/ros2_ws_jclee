#user_input_query_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk

class UserTextInputNode(Node):
    def __init__(self):
        super().__init__('user_text_input_node')
        self.publisher_ = self.create_publisher(String, '/user_text_input', 10)

        # Tkinter GUI config
        self.window = tk.Tk()
        self.window.title("User Text Input")

        self.label = tk.Label(self.window, text="Enter query for YOLOWorld:")
        self.label.pack(pady=5)

        self.entry = tk.Entry(self.window, width=30)
        self.entry.pack(padx=5, pady=5)

        self.button = tk.Button(self.window, text="Query Send", command=self.send_text)
        self.button.pack(pady=5)

    def send_text(self):
        # publishing the extracted text
        text_to_send = self.entry.get()
        if text_to_send.strip():
            msg = String()
            msg.data = text_to_send
            self.publisher_.publish(msg)
            self.get_logger().info(f"Sent text: {text_to_send}")
        else:
            self.get_logger().warn("Attemted to send empty text...!!")

    def run_gui(self):
        self.window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = UserTextInputNode()

    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
