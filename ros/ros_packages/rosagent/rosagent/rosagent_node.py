#!/usr/bin/env python3
import rclpy

from rosagent.operation.rosagent import RosAgent

def main(args=None):
    rclpy.init(args=args)
    
    rosagent = RosAgent()
    rosagent.run()

    rosagent.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
