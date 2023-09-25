import cv2
import numpy as np
from random import randint

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImagePublisher(Node):
    
    def __init__(self):
        super().__init__("image_publisher")
        self.publisher_ = self.create_publisher(Image, "video_frames", 10)
        timer_period = 2 # Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.br = CvBridge()
        self._line_colors = ((255,0,0), (0,255,0), (0,0,255))
   
    def timer_callback(self):
        frame = np.zeros(shape=(1920,1080,3), dtype=np.int8)
        cv2.line(frame, (0,0), (1920,1080), self._line_colors[randint(0,2)], 50)
        self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
