#!/usr/bin/env python3

import rclpy
from autonav_msgs.msg import ControllerInput, MotorInput
from scr.node import Node
from scr.states import DeviceStateEnum, SystemStateEnum, SystemModeEnum
from scr.states import SystemStateEnum

#!/usr/bin/env python3

import rclpy
import numpy as np
from scr.node import Node
from autonav_msgs.msg import ControllerInput

class Manual24Node(Node):

    def __init__(self):
        super().__init__('manual_24')

    def init(self):
        self.max_forward_speed = 1
        self.max_angular_speed = np.pi/4

        self.controller_state = {}

        self.set_device_state(DeviceStateEnum.STANDBY)
        
        self.get_logger().info("HERE")

        self.controllerSubscriber = self.create_subscription(
            ControllerInput,
            '/autonav/controller_input',
            self.input_callback,
            10)
        
        self.motorPublisher = self.create_publisher(
            MotorInput,
            '/autonav/MotorInput',
            10
        )

        self.controllerSubscriber  # prevent unused variable warning


    def input_callback(self, msg):
        self.set_device_state(DeviceStateEnum.OPERATING)
        self.deserialize_controller_state(msg)
        self.get_logger().info(f"I heard: {str(self.controller_state)}")


        self.change_system_state()
        self.compose_motorinput_message()


    def deserialize_controller_state(self, msg):
        attributes = [n for n in dir(msg) if not (n.startswith('__') or n.startswith('_'))]
        attributes.remove('SLOT_TYPES')
        attributes.remove('get_fields_and_field_types')
        for attribute in attributes:
            self.controller_state[attribute] = getattr(msg, attribute)


    def normalize(self, input, output_start, output_end, input_start, input_end):
        return output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
    

    def change_system_state(self):
        if self.controller_state['btn_start'] == 1.0:
            new_system_state = SystemStateEnum.MANUAL
            self.get_logger().info("CHANGING SYSTEM STATE")
            self.set_system_state(new_system_state)

        elif self.controller_state['btn_select'] == 1.0:
            new_system_state = SystemStateEnum.DISABLED
            self.get_logger().info("CHANGING SYSTEM STATE")
            self.set_system_state(new_system_state)


    def compose_motorinput_message(self):
        forward_velocity = self.normalize(self.controller_state["abs_gas"] - self.controller_state["abs_brake"], -self.max_forward_speed, self.max_forward_speed, -1, 1)
        angular_velocity = self.normalize(self.controller_state["abs_x"], self.max_angular_speed, -self.max_angular_speed, -1, 1)

        motor_msg = MotorInput()
        motor_msg.forward_velocity = forward_velocity
        motor_msg.angular_velocity = angular_velocity

        self.motorPublisher.publish(motor_msg)


def main(args=None):
    # rclpy.init(args=args)

    # controller_listener = Manual24()

    # rclpy.spin(controller_listener)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # controller_listener.destroy_node()
    # rclpy.shutdown()
    rclpy.init()
    node = Manual24Node()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()