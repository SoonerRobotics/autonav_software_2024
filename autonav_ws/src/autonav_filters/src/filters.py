#!/usr/bin/env python3

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


CONFIG_FILTER_TYPE = "filter_type"
CONFIG_DEGREE_OFFSET = "degree_offset"
CONFIG_SEED_HEADING = "seed_heading"


class FiltersNodeConfig:
    def __init__(self):
        self.filterType = FilterType.PARTICLE_FILTER
        self.degreeOffset = 107.0
        self.seedHeading = False


class FiltersNode(Node):
    def __init__(self):
        super().__init__("autonav_filters")

        self.lastIMUReceived = None
        self.firstGps = None
        self.lastGps = None
        
        self.latitudeLength = self.declare_parameter("latitude_length", 111086.2).get_parameter_value().double_value
        self.longitudeLength = self.declare_parameter("longitude_length", 81978.2).get_parameter_value().double_value
        
        self.pf = ParticleFilter(self.latitudeLength, self.longitudeLength)
        self.reckoning = DeadReckoningFilter()
        self.config = FiltersNodeConfig()
        
        self.onReset()

    def init(self):
        self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.create_subscription(IMUData, "/autonav/imu", self.onIMUReceived, 20);
        self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.positionPublisher = self.create_publisher(Position, "/autonav/position", 20)

        self.set_device_state(DeviceStateEnum.OPERATING)
    
    def onIMUReceived(self, msg: IMUData):
        self.lastIMUReceived = msg
        
    def getRealHeading(self, heading: float):
        if heading < 0:
            heading = 360 + -heading
        
        heading += self.config.degreeOffset
        return heading

    def onReset(self):
        if self.lastIMUReceived is not None and self.config.seedHeading:
            self.reckoning.reset(self.getRealHeading(self.lastIMUReceived.heading))
            self.pf.init_particles(self.getRealHeading(self.lastIMUReceived.heading), True)
        else:
            self.reckoning.reset()
            self.pf.init_particles()

    def system_state_transition(self, old: SystemState, updated: SystemState):
        if old.state != SystemStateEnum.AUTONOMOUS and updated.state == SystemStateEnum.AUTONOMOUS:
            self.onReset()

        if old.mobility == False and updated.mobility == True:
            self.onReset()
            
    def onGPSReceived(self, msg: GPSFeedback):
        if msg.gps_fix == 0 and msg.is_locked == False:
            return
        
        if self.firstGps is None:
            self.firstGps = msg

        self.lastGps = msg

        filterType = self.config.filterType
        if filterType == FilterType.PARTICLE_FILTER:
            self.pf.gps(msg)
        elif filterType == FilterType.DEAD_RECKONING:
            self.reckoning.gps(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        filterType = self.config.filterType
        averages = None
        if filterType == FilterType.PARTICLE_FILTER:
            averages = self.pf.feedback(msg)
        if filterType == FilterType.DEAD_RECKONING:
            averages = self.reckoning.feedback(msg)
            
        if averages is None:
            return
            
        position = Position()
        position.x = averages[0]
        position.y = averages[1]
        position.theta = (-1 * math.pi * 2 + averages[2]) * 1

        
        if self.firstGps is not None:
            gps_x = self.firstGps.latitude + position.x / self.latitudeLength
            gps_y = self.firstGps.longitude - position.y / self.longitudeLength
            position.latitude = gps_x
            position.longitude = gps_y

        if self.system_mode == SystemModeEnum.SIMULATION and self.lastGps is not None:
            position.latitude = self.lastGps.latitude
            position.longitude = self.lastGps.longitude
        
        self.positionPublisher.publish(position)


def main():
    rclpy.init()
    node = FiltersNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
