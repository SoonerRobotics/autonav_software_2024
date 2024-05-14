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
        self.output_width = 480
        self.output_height = 640
        self.camera_index_left = 0
        self.camera_index_right = 2
        self.scan_rate = 1.0


class CameraNode(Node):
    def __init__(self):
        super().__init__("autonav_serial_camera")
        self.camera_publisher_left = self.create_publisher(CompressedImage, "/autonav/camera/compressed/left", 20)
        self.camera_publisher_right = self.create_publisher(CompressedImage, "/autonav/camera/compressed/right", 20)

    def init(self):
        self.get_logger().info("Initializing camera node...")
        self.create_threads()

    def create_threads(self):
        self.camera_thread_left = threading.Thread(target=self.camera_worker, args=("left",))
        self.camera_thread_left.daemon = True
        self.camera_thread_left.start()

        self.camera_thread_right = threading.Thread(target=self.camera_worker, args=("right",))
        self.camera_thread_right.daemon = True
        self.camera_thread_right.start()

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return CameraNodeConfig()

    def camera_worker(self, *args, **kwargs):
        index_name = args[0] if len(args) > 0 else ""
        camera_index = self.config.camera_index_left if index_name == "left" else self.config.camera_index_right

        capture = None
        while rclpy.ok() and self.system_state != SystemStateEnum.SHUTDOWN:
            try:
                if not os.path.exists("/dev/video" + str(camera_index)):
                    time.sleep(self.config.scan_rate)
                    continue

                capture = cv2.VideoCapture(camera_index)
                if capture is None or not capture.isOpened():
                    time.sleep(self.config.scan_rate)
                    continue

                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.output_width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.output_height)
                # self.set_device_state(DeviceStateEnum.OPERATING)
            except:
                # self.set_device_state(DeviceStateEnum.STANDBY)
                time.sleep(self.config.scan_rate)
                continue

            while rclpy.ok() and self.system_state != SystemStateEnum.SHUTDOWN:
                # if self.device_state != DeviceStateEnum.OPERATING:
                #     continue

                try:
                    ret, frame = capture.read()
                    if index_name == "left":
                        frame = cv2.flip(frame, 1)
                        frame = cv2.flip(frame, 0)
                    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                    frame = cv2.resize(frame, (self.config.output_width, self.config.output_height))
                except:
                    if capture is not None:
                        capture.release()
                        capture = None

                    # self.set_device_state(DeviceStateEnum.STANDBY)
                    break

                if not ret or frame is None:
                    continue

                if index_name == "left":
                    self.camera_publisher_left.publish(bridge.cv2_to_compressed_imgmsg(frame))
                else:
                    self.camera_publisher_right.publish(bridge.cv2_to_compressed_imgmsg(frame))
                time.sleep(1.0 / self.config.refresh_rate)


def main():
    rclpy.init()
    node = CameraNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()