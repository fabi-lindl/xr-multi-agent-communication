import rclpy
from rclpy.node import Node

from unity_robotics_demo_msgs.msg import PosRot

class PoseSubscriber(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(
            PosRot, "pos_rot", self.listener_callback, 10)
        self.subscription # Prevent unused variable warning.
    
    def listener_callback(self, msg):
        self.get_logger().info("Received pose message:")
        self.get_logger().info(f"pos_x = {msg.pos_x}")
        self.get_logger().info(f"pos_y = {msg.pos_y}")
        self.get_logger().info(f"pos_z = {msg.pos_z}")
        self.get_logger().info(f"rot_x = {msg.rot_x}")
        self.get_logger().info(f"rot_y = {msg.rot_y}")
        self.get_logger().info(f"rot_z = {msg.rot_z}")
        self.get_logger().info(f"rot_w = {msg.rot_w}")
        self.get_logger().info("----------------------")
    
def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
