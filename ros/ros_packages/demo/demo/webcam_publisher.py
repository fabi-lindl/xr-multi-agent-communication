import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WebcamPublisher(Node):

    def __init__(self):
        super().__init__("webcam_publisher")
        self.publisher_ = self.create_publisher(Image, "video_frames", 10)
        timer_period = 0.2 # Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info("[Success] Webcame image published!")
        else:
            self.get_logger().info("[Error] Webcam image not published!")

def main(args=None):
    rclpy.init(args=args)
    image_publisher = WebcamPublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
