#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autonav_msgs.msg import MotorInput


class ControllerListener(Node):

    def __init__(self):
        super().__init__('motor_subscriber')

        self.controller_state = {}

        self.subscription = self.create_subscription(
            MotorInput,
            '/autonav/MotorInput',
            self.listener_callback,
            10)
        
        self.subscription # prevent unused variable warning


    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.forward_velocity}, {msg.angular_velocity}")


    def deserialize_controller_state(self, msg):
        attributes = [n for n in dir(msg) if not (n.startswith('__') or n.startswith('_'))]
        attributes.remove('SLOT_TYPES')
        attributes.remove('get_fields_and_field_types')
        for attribute in attributes:
            self.controller_state[attribute] = getattr(msg, attribute)


def main(args=None):
    rclpy.init(args=args)

    controller_listener = ControllerListener()

    rclpy.spin(controller_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()