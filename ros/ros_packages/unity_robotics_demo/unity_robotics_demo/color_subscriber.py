import rclpy
from rclpy.node import Node

from unity_robotics_demo_msgs.msg import UnityColor

class ColorSubscriberNode(Node):

    def __init__(self):
        super().__init__("subscriber_node")
        self.subscriber = self.create_subscription(UnityColor, 'color',
                                                   self.callback, 10)

    def callback(self, message):
        self.get_logger().info('I heard: "%s"' % message)

def main(args=None):
    rclpy.init(args=args)
    node = ColorSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
