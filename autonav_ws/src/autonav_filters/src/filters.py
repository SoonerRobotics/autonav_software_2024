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
        self.degree_offset = 107.0
        self.seed_heading = False


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
        self.config = self.get_default_config()
        
        self.onReset()

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return FiltersNodeConfig()

    def init(self):
        self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.create_subscription(IMUData, "/autonav/imu", self.onIMUReceived, 20)
        self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.positionPublisher = self.create_publisher(Position, "/autonav/position", 20)

        self.set_device_state(DeviceStateEnum.OPERATING)
    
    def onIMUReceived(self, msg: IMUData):
        self.lastIMUReceived = msg
        
    def getRealHeading(self, heading: float):
        if heading < 0:
            heading = 360 + -heading
        
        heading += self.config.degree_offset
        return heading

    def onReset(self):
        if self.lastIMUReceived is not None and self.config.seed_heading:
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
        #self.get_logger().info("RECEIVED GPS")
        if msg.gps_fix == 0 and msg.is_locked == False:
            return
        
        if self.firstGps is None:
            self.firstGps = msg

        #self.get_logger().info(f"firstGps.latitude {self.firstGps.latitude}\n")
        #self.get_logger().info(f"firstGps.longitude {self.firstGps.longitude}\n")

        self.lastGps = msg

        filterType = self.config.filter_type
        if filterType == FilterType.PARTICLE_FILTER:
            self.pf.gps(msg)
        '''elif filterType == FilterType.DEAD_RECKONING:
            self.reckoning.gps(msg)
        '''
    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        filterType = self.config.filter_type
        averages = None
        if filterType == FilterType.PARTICLE_FILTER:
            averages = self.pf.feedback(msg)
        '''if filterType == FilterType.DEAD_RECKONING:
            averages = self.reckoning.feedback(msg)
        '''    
        if averages is None:
            return
            
        position = Position()
        position.x = averages[0]
        position.y = averages[1]
        #self.get_logger().info(f"position.x {position.x}\n")
        #self.get_logger().info(f"position.y {position.y}\n")
        position.theta = (-1 * math.pi * 2 + averages[2]) * 1
        #self.get_logger().info(f"position.theta {position.theta}\n")
        
        if self.firstGps is not None:
            gps_x = self.firstGps.latitude + position.x / self.latitudeLength
            gps_y = self.firstGps.longitude - position.y / self.longitudeLength
            #self.get_logger().info(f"firstGps.latitude {self.firstGps.latitude}\n")
            #self.get_logger().info(f"firstGps.longitude {self.firstGps.longitude}\n")
            #self.get_logger().info(f"gps_x {gps_x}\n")
            #self.get_logger().info(f"gps_y {gps_y}\n")
            
            position.latitude = gps_x
            #self.get_logger().info(f"position.latitude {position.latitude}\n")
            position.longitude = gps_y
            #self.get_logger().info(f"position.longitude {position.longitude}\n")

        if self.system_mode == SystemModeEnum.SIMULATION and self.lastGps is not None:
            #position.latitude = self.lastGps.latitude
            #position.longitude = self.lastGps.longitude
            print("do nothing")
        
        #self.get_logger().info(f"averages 0: {averages[0]}, averages 1: {averages[1]}, averages 2: {averages[2]}")
        #self.get_logger().info(f"publishing position x: {position.x}, y: {position.y}, {position.theta}")
        self.get_logger().info(f"position latitude: {position.latitude}, longitude {position.longitude}")
        self.positionPublisher.publish(position)


def main():
    rclpy.init()
    node = FiltersNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
