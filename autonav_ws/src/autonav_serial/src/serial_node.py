#!/usr/bin/env python3

from ctypes import Structure, c_bool, c_uint8
import rclpy
import time
import can
import threading
import struct
from autonav_msgs.msg import MotorInput, MotorFeedback, MotorControllerDebug, SafetyLights, Conbus
from scr.node import Node
from scr.states import DeviceStateEnum, SystemStateEnum, SystemStateEnum


MOTOR_CONTROL_ID = 10
ESTOP_ID = 0
MOBILITY_STOP_ID = 1
MOBILITY_START_ID = 9
MOTOR_FEEDBACK_ID = 14
SAFETY_LIGHTS_ID = 13

CAN_50 = 50
CAN_51 = 51


class SafetyLightsPacket(Structure):
    _fields_ = [
        ("autonomous", c_bool, 1),
        ("eco", c_uint8, 1),
        ("mode", c_uint8, 6),
        ("brightness", c_uint8, 8),
        ("red", c_uint8, 8),
        ("green", c_uint8, 8),
        ("blue", c_uint8, 8),
        ("blink_period", c_uint8, 8)
    ]


class SerialMotors(Node):
    def __init__(self):
        super().__init__("autonav_serial_can")

        self.currentForwardVel = 0.0
        self.setpointForwardVel = 0.0
        self.currentAngularVel = 0.0
        self.setpointAngularVel = 0.0
        self.can = None
        self.lastMotorInput = None
        self.duplicateIterations = []

        self.safetyLightsSubscriber = self.create_subscription(SafetyLights, "/autonav/SafetyLights", self.onSafetyLightsReceived, 20)
        self.motorInputSubscriber = self.create_subscription(MotorInput, "/autonav/MotorInput", self.onMotorInputReceived, 20)
        self.motorDebugPublisher = self.create_publisher(MotorControllerDebug, "/autonav/MotorControllerDebug", 20)
        self.motorFeedbackPublisher = self.create_publisher(MotorFeedback, "/autonav/MotorFeedback", 20)
        self.conbuSubscriber = self.create_subscription(Conbus, "/autonav/conbus/instruction", self.onConbusReceived, 20)
        self.conbusPublisher = self.create_publisher(Conbus, "/autonav/conbus/data", 20)

    def init(self):
        self.canTimer = self.create_timer(0.5, self.canWorker)
        self.canReadThread = threading.Thread(target=self.canThreadWorker)
        self.canReadThread.daemon = True
        self.canReadThread.start()

    def zero_motors(self):
        packed_data = struct.pack("hh", int(0 * 1000.0), int(0 * 1000.0))
        can_msg = can.Message(arbitration_id=MOTOR_CONTROL_ID, data=packed_data)
        try:
            self.can.send(can_msg)
        except can.CanError:
            pass

    def system_state_transition(self, old: SystemStateEnum, updated: SystemStateEnum):
        if old.state != SystemStateEnum.DISABLED and updated.state == SystemStateEnum.DISABLED:
            self.zero_motors()
        
        if old.state == SystemStateEnum.AUTONOMOUS and updated.state == SystemStateEnum.MANUAL:
            self.zero_motors()

        # If we enter autonomous mode, we need to send a stop message to the motors
        if old.state != SystemStateEnum.AUTONOMOUS and updated.state == SystemStateEnum.AUTONOMOUS:
            self.set_system_mobility(False)
            can_msg = can.Message(arbitration_id=MOBILITY_STOP_ID, data=bytes([0]))
            try:
                self.can.send(can_msg)
            except can.CanError:
                pass

        if old.state == SystemStateEnum.AUTONOMOUS and updated.state != SystemStateEnum.AUTONOMOUS:
            self.set_system_mobility(False)
            can_msg = can.Message(arbitration_id=MOBILITY_STOP_ID, data=bytes([0]))
            try:
                self.can.send(can_msg)
            except can.CanError:
                pass

        if old.state == SystemStateEnum.MANUAL and updated.state == SystemStateEnum.AUTONOMOUS:
            self.zero_motors()

        if old.mobility == True and updated.mobility == False:
            self.zero_motors()

    def canThreadWorker(self):
        while rclpy.ok():
            if self.device_state != DeviceStateEnum.READY and self.device_state != DeviceStateEnum.OPERATING:
                continue
            if self.can is not None:
                try:
                    msg = self.can.recv(timeout=1)
                    if msg is not None:
                        self.onCanMessageReceived(msg)
                except can.CanError:
                    pass

    def getClockMs(self):
        return time.time() * 1000.0

    def onCanMessageReceived(self, msg):
        arb_id = msg.arbitration_id
        if arb_id == MOTOR_FEEDBACK_ID:
            deltaX, deltaY, deltaTheta = struct.unpack("hhh", msg.data)
            feedback = MotorFeedback()
            feedback.delta_theta = deltaTheta / 10000.0
            feedback.delta_y = deltaY / 10000.0
            feedback.delta_x = deltaX / 10000.0
            self.motorFeedbackPublisher.publish(feedback)

        if arb_id == ESTOP_ID:
            self.set_system_mobility(False)

        if arb_id == MOBILITY_STOP_ID:
            self.set_system_mobility(False)

        if arb_id == MOBILITY_START_ID:
            self.set_system_mobility(True)

        if arb_id == CAN_50:
            currentForwardVel, setpointForwardVel, currentAngularVel, setpointAngularVel = struct.unpack(
                "hhhh", msg.data)
            self.currentForwardVel = currentForwardVel / 1000.0
            self.setpointForwardVel = setpointForwardVel / 1000.0
            self.currentAngularVel = currentAngularVel / 1000.0
            self.setpointAngularVel = setpointAngularVel / 1000.0

        if arb_id == CAN_51:
            leftMotorOutput, rightMotorOutput = struct.unpack("hh", msg.data)
            leftMotorOutput /= 1000.0
            rightMotorOutput /= 1000.0

            # Create a MotorControllerDebug message and publish it
            pkg = MotorControllerDebug()
            pkg.current_forward_velocity = self.currentForwardVel
            pkg.forward_velocity_setpoint = self.setpointForwardVel
            pkg.current_angular_velocity = self.currentAngularVel
            pkg.angular_velocity_setpoint = self.setpointAngularVel
            pkg.left_motor_output = leftMotorOutput
            pkg.right_motor_output = rightMotorOutput
            pkg.timestamp = self.getClockMs() * 1.0
            self.motorDebugPublisher.publish(pkg)

        if arb_id >= 1000 and arb_id < 1400:
            # self.log(f"[CAN -> {arb_id}] Received ConBus message")
            pkg = Conbus()
            pkg.id = arb_id
            pkg.data = msg.data
            self.conbusPublisher.publish(pkg)

    def canWorker(self):
        try:
            with open("/dev/autonav-can-835", "r") as f:
                pass

            if self.can is not None:
                return

            self.can = can.ThreadSafeBus(
                bustype="slcan", channel="/dev/autonav-can-835", bitrate=100000)
            self.set_device_state(DeviceStateEnum.OPERATING)
        except:
            if self.can is not None:
                self.can = None

            if self.device_state != DeviceStateEnum.STANDBY:
                self.set_device_state(DeviceStateEnum.STANDBY)

    def onSafetyLightsReceived(self, lights: SafetyLights):
        if self.device_state != DeviceStateEnum.OPERATING:
            return

        packed_data = SafetyLightsPacket()
        packed_data.autonomous = lights.autonomous
        packed_data.eco = False
        packed_data.mode = 0
        packed_data.brightness = lights.brightness
        packed_data.red = lights.red
        packed_data.green = lights.green
        packed_data.blue = lights.blue
        packed_data.blink_period = 500
        can_msg = can.Message(
            arbitration_id=SAFETY_LIGHTS_ID, data=bytes(packed_data))
        try:
            self.can.send(can_msg)
        except can.CanError:
            pass

    def onConbusReceived(self, instruction: Conbus):
        if self.device_state != DeviceStateEnum.OPERATING:
            return

        try:
            actual_bytes = bytes(instruction.data)
            can_msg = can.Message(
                arbitration_id=instruction.id, data=actual_bytes)
            try:
                self.can.send(can_msg)
            except can.CanError:
                pass
        except:
            pass

    def onMotorInputReceived(self, input: MotorInput):
        if self.device_state != DeviceStateEnum.OPERATING:
            return

        packed_data = struct.pack("hh", int(input.forward_velocity * 1000.0), int(input.angular_velocity * 1000.0))
        can_msg = can.Message(arbitration_id=MOTOR_CONTROL_ID, data=packed_data)
        try:
            self.can.send(can_msg)
        except can.CanError:
            pass


def main():
    rclpy.init()
    node = SerialMotors()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
