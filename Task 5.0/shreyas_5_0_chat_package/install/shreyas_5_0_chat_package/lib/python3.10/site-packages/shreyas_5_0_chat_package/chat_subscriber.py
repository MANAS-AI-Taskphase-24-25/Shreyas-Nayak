import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatSubscriber(Node):
    def __init__(self):
        super().__init__('chat_subscriber')
        self.subscription = self.create_subscription(String, 'chat_topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ChatSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
