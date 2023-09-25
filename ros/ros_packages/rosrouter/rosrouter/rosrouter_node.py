#/usr/bin/env python3
import rclpy

from rosrouter.router.rosrouter import RosRouter

def main(args=None):
    rclpy.init(args=args)

    rosrouter = RosRouter()
    rosrouter.run()
    rclpy.spin(rosrouter)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
