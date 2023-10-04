#!/usr/bin/env python3

import rclpy

from autonav_core.core import AutonavNode
from autonav_msgs.msg import Test


class ExampleNode(AutonavNode):
    def __init__(self):
        super().__init__("example_python_node")

        # Create a publisher for the Test message, with a queue size of 10, and a topic name of "/autonav/test"
        self.test_subscriber = self.create_subscription(Test, "/autonav/test", self.test_callback, 10)

    def test_callback(self, msg: Test):
        self.get_logger().info("Received message: " + str(msg.data))


def main():
    rclpy.init()
    rclpy.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
