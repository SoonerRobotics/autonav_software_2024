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
import subprocess

bridge = CvBridge()


class CameraNodeConfig:
    def __init__(self):
        self.refresh_rate = 8
        self.output_width = 480
        self.output_height = 640
        self.scan_rate = 1.0
        self.flip_horizontal = False
        self.flip_vertical = False
        self.rotate_clockwise = False


class CameraNode(Node):
    def __init__(self, side, udev_path):
        super().__init__("autonav_serial_camera_" + side)
        self.camera_publisher = self.create_publisher(CompressedImage, "/autonav/camera/compressed/" + side, self.qos_profile)
        self.camera_thread = None
        self.camera_kill = False
        self.camera_side = side
        self.camera_path = udev_path

    def init(self):
        self.create_thread()
        
    def create_thread(self):
        if self.camera_thread is not None:
            self.camera_kill = True
            self.camera_thread.join()
            self.camera_thread = None
            
        self.camera_kill = False
        self.camera_thread = threading.Thread(target=self.camera_worker)
        self.camera_thread.start()

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))
        self.create_thread()

    def get_default_config(self):
        return CameraNodeConfig()

    def camera_worker(self):
        capture = None
        while rclpy.ok() and self.system_state != SystemStateEnum.SHUTDOWN and not self.camera_kill:
            try:
                if not os.path.exists(self.camera_path):
                    time.sleep(self.config.scan_rate)
                    continue

                capture = cv2.VideoCapture(self.camera_path)
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

            while rclpy.ok() and self.system_state != SystemStateEnum.SHUTDOWN and not self.camera_kill:
                if self.device_state != DeviceStateEnum.OPERATING:
                    continue

                try:
                    ret, frame = capture.read()
                    
                    if self.config.rotate_clockwise:
                        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                    else:
                        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    
                    frame = cv2.resize(frame, (self.config.output_width, self.config.output_height))
                    
                    if self.config.flip_horizontal:
                        frame = cv2.flip(frame, 1)
                        
                    if self.config.flip_vertical:
                        frame = cv2.flip(frame, 0)
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

    # Check whether index0 or index1 is the actual footage. Use V4L2-CTL to check if "exposure_time_absolute" exists
    # sudo v4l2-ctl --device={x} --all
    left_index0 = subprocess.run(["v4l2-ctl", "--device=0", "--all"], stdout=subprocess.PIPE)
    right_index0 = subprocess.run(["v4l2-ctl", "--device=2", "--all"], stdout=subprocess.PIPE)

    correct_left = None
    correct_right = None
    if "exposure_time_absolute" in left_index0.stdout.decode("utf-8"):
        correct_left = 0
    else:
        correct_left = 1
        
    if "exposure_time_absolute" in right_index0.stdout.decode("utf-8"):
        correct_right = 0
    else:
        correct_right = 1

    node_left = CameraNode("left", "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_8174526F-video-index" + str(correct_left))
    node_right = CameraNode("right", "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_3F47331F-video-index" + str(correct_right))
    Node.run_nodes([node_left, node_right])
    rclpy.shutdown()


if __name__ == "__main__":
    main()