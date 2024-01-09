#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position, SafetyLights
from scr_msgs.msg import SystemState
from scr.node import Node
from scr.states import DeviceStateEnum, SystemStateEnum
from scr_core import clamp
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
import math
import rclpy


IS_SOUTH = False
BACK_SPEED = 0.40


def hexToRgb(color: str):
    if color[0] == "#":
        color = color[1:]
    return [int(color[0:2], 16), int(color[2:4], 16), int(color[4:6], 16)]


def toSafetyLights(autonomous: bool, eco: bool, mode: int, brightness: int, color: str) -> SafetyLights:
    pkg = SafetyLights()
    pkg.mode = mode
    pkg.autonomous = autonomous
    pkg.eco = eco
    pkg.brightness = brightness
    colorr = hexToRgb(color)
    pkg.red = colorr[0]
    pkg.green = colorr[1]
    pkg.blue = colorr[2]
    return pkg


class PathResolverNodeConfig:
    def __init__(self):
        self.FORWARD_SPEED = 2.1
        self.REVERSE_SPEED = -0.4
        self.RADIUS_MULTIPLIER = 1.2
        self.RADIUS_MAX = 4.0
        self.RADIUS_START = 0.7
        self.ANGULAR_AGGRESSINON = 2.2
        self.MAX_ANGULAR_SPEED = 0.5


class PathResolverNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_resolver")
        self.position = Position()

    def init(self):
        self.pure_pursuit = PurePursuit()
        self.backCount = -1
        self.status = -1
        self.path_subscriber = self.create_subscription(Path, "/autonav/path", self.on_path_received, 20)
        self.position_subscriber = self.create_subscription(Position, "/autonav/position", self.on_position_received, 20)
        self.motor_publisher = self.create_publisher(MotorInput, "/autonav/MotorInput", 20)
        self.safety_lights_publisher = self.create_publisher(SafetyLights, "/autonav/SafetyLights", 20)
        self.config = PathResolverNodeConfig()

        self.create_timer(0.05, self.onResolve)
        self.set_device_state(DeviceStateEnum.READY)

    def reset(self):
        self.position = None
        self.backCount = -1

    def system_state_transition(self, old: SystemState, updated: SystemState):
        if updated.state == SystemStateEnum.AUTONOMOUS and self.device_state == DeviceStateEnum.READY:
            self.set_device_state(DeviceStateEnum.OPERATING)

        if updated.state != SystemStateEnum.AUTONOMOUS and self.device_state == DeviceStateEnum.OPERATING:
            self.set_device_state(DeviceStateEnum.READY)
            inputPacket = MotorInput()
            inputPacket.forward_velocity = 0.0
            inputPacket.angular_velocity = 0.0
            self.motor_publisher.publish(inputPacket)

        if updated.state == SystemStateEnum.AUTONOMOUS and self.device_state == DeviceStateEnum.OPERATING and updated.mobility == False:
            self.safety_lights_publisher.publish(toSafetyLights(False, False, 2, 255, "#00A36C"))

        if updated.state == SystemStateEnum.AUTONOMOUS and self.device_state == DeviceStateEnum.OPERATING and updated.mobility == True:
            self.safety_lights_publisher.publish(toSafetyLights(True, False, 2, 255, "#FFFFFF"))

    def on_position_received(self, msg):
        self.position = msg
        self.position.x = 0.0
        self.position.y = 0.0
        self.position.theta = 0.0

    def get_angle_difference(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return delta

    def on_path_received(self, msg: Path):
        self.points = [x.pose.position for x in msg.poses]
        self.pure_pursuit.set_points([(point.x, point.y)for point in self.points])

    def onResolve(self):
        if self.position is None or self.device_state != DeviceStateEnum.OPERATING or self.system_state != SystemStateEnum.AUTONOMOUS or not self.mobility:
            return

        cur_pos = (self.position.x, self.position.y)
        lookahead = None
        radius = self.config.RADIUS_START
        while lookahead is None and radius <= self.config.RADIUS_MAX:
            lookahead = self.pure_pursuit.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
            radius *= self.config.RADIUS_MULTIPLIER

        motor_packet = MotorInput()
        motor_packet.forward_velocity = 0.0
        motor_packet.angular_velocity = 0.0

        if self.backCount == -1 and (lookahead is not None and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.25):
            angle_diff = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])
            error = self.get_angle_difference(angle_diff, self.position.theta) / math.pi
            forward_speed = self.config.FORWARD_SPEED * (1 - abs(error)) ** 5
            motor_packet.forward_velocity = forward_speed
            motor_packet.angular_velocity = clamp(error * self.config.ANGULAR_AGGRESSINON, -self.config.MAX_ANGULAR_SPEED, self.config.MAX_ANGULAR_SPEED)

            if self.status == 0:
                self.safety_lights_publisher.publish(toSafetyLights(True, False, 2, 255, "#FFFFFF"))
                self.status = 1
        else:
            if self.backCount == -1:
                self.backCount = 8
            else:
                self.safety_lights_publisher.publish(toSafetyLights(True, False, 2, 255, "#FF0000"))
                self.status = 0
                self.backCount -= 1

            motor_packet.forward_velocity = self.config.REVERSE_SPEED
            motor_packet.angular_velocity = BACK_SPEED if IS_SOUTH else (-1 * BACK_SPEED)

        if not self.mobility:
            motor_packet.forward_velocity = 0.0
            motor_packet.angular_velocity = 0.0

        self.motor_publisher.publish(motor_packet)


def main():
    rclpy.init()
    node = PathResolverNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
