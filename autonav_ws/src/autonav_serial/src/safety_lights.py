#!/usr/bin/env python3

from ctypes import Structure, c_bool, c_uint8
from scr.states import DeviceStateEnum
from autonav_msgs.msg import SafetyLights
from scr.node import Node
import threading
import serial
import rclpy
import json
import os
import time


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


class SafetyLightsSerial(Node):
    def __init__(self):
        super().__init__("autonav_serial_safetylights")
        self.safetyLightsSubscriber = self.create_subscription(
            SafetyLights, "/autonav/SafetyLights", self.onSafetyLightsReceived, 20)
        self.pico = None
        self.writeQueue = []
        self.writeQueueLock = threading.Lock()
        self.writeThread = threading.Thread(target=self.picoWriteWorker)
        self.writeThread.daemon = True

    def init(self):
        self.writeThread.start()

    def picoWriteWorker(self):
        while rclpy.ok():
            does_exist = os.path.exists("/dev/autonav-mc-safetylights")
            if not does_exist:
                time.sleep(1)
                continue

            self.pico = serial.Serial(
                "/dev/autonav-mc-safetylights", baudrate=115200)
            self.set_device_state(DeviceStateEnum.OPERATING)
            while self.pico is not None and self.pico.is_open:
                if not self.pico.is_open or self.pico.in_waiting > 0 or len(self.writeQueue) == 0:
                    continue

                self.writeQueueLock.acquire()
                if len(self.writeQueue) > 0:
                    jsonStr = json.dumps(self.writeQueue.pop(0))
                    try:
                        self.pico.write(bytes(jsonStr, "utf-8"))
                    except:
                        self.log(f"Failed to write to serial port: {jsonStr}")
                self.writeQueueLock.release()

    def onSafetyLightsReceived(self, lights: SafetyLights):
        data = {}
        data["autonomous"] = lights.autonomous
        data["eco"] = lights.eco
        data["mode"] = lights.mode
        data["brightness"] = lights.brightness
        data["red"] = lights.red
        data["green"] = lights.green
        data["blue"] = lights.blue
        data["blink_period"] = 500 / 10

        self.writeQueueLock.acquire()
        self.writeQueue.append(data)
        self.writeQueueLock.release()


def main():
    rclpy.init()
    node = SafetyLightsSerial()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
