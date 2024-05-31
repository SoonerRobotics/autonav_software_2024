#!/usr/bin/env python3

import json
from types import SimpleNamespace
from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position, IMUData
from scr.states import DeviceStateEnum, SystemStateEnum, SystemModeEnum
from particlefilter import ParticleFilter
from deadrekt import DeadReckoningFilter
from scr_msgs.msg import SystemState
from scr.node import Node
from enum import IntEnum
import rclpy
import math


class FilterType(IntEnum):
    DEAD_RECKONING = 0,
    PARTICLE_FILTER = 1


class FiltersNodeConfig:
    def __init__(self):
        self.filter_type = FilterType.PARTICLE_FILTER


class FiltersNode(Node):
    def __init__(self):
        super().__init__("autonav_filters")

        self.first_gps = None
        self.last_gps = None

        self.latitude_length = self.declare_parameter("latitude_length", 111086.2).get_parameter_value().double_value
        self.longitude_length = self.declare_parameter("longitude_length", 81978.2).get_parameter_value().double_value

        self.pf = ParticleFilter(self.latitude_length, self.longitude_length)
        self.dr = DeadReckoningFilter()
        self.config = self.get_default_config()

        self.onReset()

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return FiltersNodeConfig()

    def init(self):
        self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.position_publisher = self.create_publisher(Position, "/autonav/position", 20)

        self.set_device_state(DeviceStateEnum.OPERATING)

    def onReset(self):
        self.dr.reset()
        self.pf.init_particles()

    def system_state_transition(self, old: SystemState, updated: SystemState):
        if old.state != SystemStateEnum.AUTONOMOUS and updated.state == SystemStateEnum.AUTONOMOUS:
            self.onReset()

        if old.mobility == False and updated.mobility == True:
            self.onReset()

    def onGPSReceived(self, msg: GPSFeedback):
        if msg.gps_fix == 0 and msg.is_locked == False:
            return

        if self.first_gps is None:
            self.first_gps = msg

        self.last_gps = msg
        if self.config.filter_type == FilterType.PARTICLE_FILTER:
            self.pf.gps(msg)
        else:
            self.dr.gps(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        averages = None
        if self.config.filter_type == FilterType.PARTICLE_FILTER:
            averages = self.pf.feedback(msg)
        else:
            averages = self.dr.feedback(msg)

        if averages is None:
            return

        position = Position()
        position.x = averages[0]
        position.y = averages[1]
        position.theta = (-1 * math.pi * 2 + averages[2]) * 1

        if self.first_gps is not None:
            gps_x = self.first_gps.latitude + position.x / self.latitude_length
            gps_y = self.first_gps.longitude - position.y / self.longitude_length
            position.latitude = gps_x
            position.longitude = gps_y

        self.position_publisher.publish(position)


def main():
    rclpy.init()
    node = FiltersNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
