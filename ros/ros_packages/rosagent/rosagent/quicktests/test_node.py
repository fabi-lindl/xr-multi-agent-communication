#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rosutility.message.system_message_routing_data import SystemMessageRoutingData
from rosutility.message.serializer.routing_data_deserializer import RoutingDataDeserializer
from rosutility.message.system_message_routing_info import SystemMessageRoutingInfo

from rosutility.logging.logger import Logger
from rosutility.logging.connection_logger import ConnectionLogger

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Hello from TestNode')
        # cl = ConnectionLogger()
        # cl.prune_expired_agentclient_connection()
        # logger = Logger()
        # logger.log_info('hi')

        # cl1 = ConnectionLogger()

        # print(cl == cl1)

        # logger1 = Logger()
        # print(logger == logger1)


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
