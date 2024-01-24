#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import time
import threading
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from scr.node import Node
from scr.states import DeviceStateEnum, SystemStateEnum
import os
import json

bridge = CvBridge()


class CameraNodeConfig:
    def __init__(self):
        self.refresh_rate = 8
        self.output_width = 640
        self.output_height = 480
        self.camera_index = 0
        self.scan_rate = 1.0


class CameraNode(Node):
    def __init__(self):
        super().__init__("autonav_serial_camera")
        self.camera_publisher = self.create_publisher(CompressedImage, "/autonav/camera/compressed", 20)
        self.camera_thread = threading.Thread(target=self.camera_worker)
        self.camera_thread.daemon = True

    def init(self):
        self.get_logger().info("Initializing camera node...")
        self.camera_thread.start()

    def config_updated(self, jsonObject):
        self.config = json.loads(json.dumps(jsonObject), object_hook=lambda d: SimpleNamespace(**d))
        self.log(json.dumps(self.config.__dict__))

    def get_default_config(self):
        return CameraNodeConfig()

    def camera_worker(self):
        capture = None
        while rclpy.ok() and self.system_state != SystemStateEnum.SHUTDOWN:
            try:
                if not os.path.exists("/dev/video" + str(self.config.camera_index)):
                    time.sleep(self.config.scan_rate)
                    continue

                capture = cv2.VideoCapture(0)
                if capture is None or not capture.isOpened():
                    time.sleep(self.config.scan_rate)
                    continue

                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.output_width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.output_height)
                self.set_device_state(DeviceStateEnum.OPERATING)
            except:
                self.set_device_state(DeviceStateEnum.STANDBY)
                time.sleep(self.config.scan_rate)
                continue

            while rclpy.ok() and self.getSystemState().state != SystemStateEnum.SHUTDOWN:
                if self.device_state != DeviceStateEnum.OPERATING:
                    continue

                try:
                    ret, frame = capture.read()
                except:
                    if capture is not None:
                        capture.release()
                        capture = None

                    self.set_device_state(DeviceStateEnum.STANDBY)
                    break

                if not ret or frame is None:
                    continue

                self.camera_publisher.publish(bridge.cv2_to_compressed_imgmsg(frame))
                time.sleep(1.0 / self.config.refresh_rate)


def main():
    rclpy.init()
    node = CameraNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
