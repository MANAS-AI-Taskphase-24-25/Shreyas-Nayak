import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatPublisher(Node):
    def __init__(self):
        super().__init__('chat_publisher')
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10)

    def send_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ChatPublisher()
    
    try:
        while rclpy.ok():
            user_input = input("Enter message: ")  # Take user input
            if user_input.lower() == "exit":
                break  # Exit if user types 'exit'
            node.send_message(user_input)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C to exit gracefully
    
    node.destroy_node()
    rclpy.shutdown()
