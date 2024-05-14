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

        self.left_flip_horizontal = True
        self.left_flip_vertical = True
        self.right_flip_horizontal = True
        self.right_flip_vertical = True

        self.rotate_left_clockwise = True
        self.rotate_right_clockwise = True
class CameraNode(Node):
    def __init__(self):
        super().__init__("autonav_serial_camera")
        self.camera_publisher_left = self.create_publisher(CompressedImage, "/autonav/camera/compressed/left", 20)
        self.camera_publisher_right = self.create_publisher(CompressedImage, "/autonav/camera/compressed/right", 20)

        self.lock_left = threading.Lock()
        self.lock_right = threading.Lock()
        self.left_kill = False
        self.right_kill = False

        self.autofocus = 0
        self.autoexposure = 0

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

    def destroy_threads(self):
        if self.camera_thread_left is None or self.camera_thread_right is None:
            return

        self.lock_left.acquire()
        self.left_kill = True
        self.lock_left.release()

        self.lock_right.acquire()
        self.right_kill = True
        self.lock_right.release()

        self.camera_thread_left.join()
        self.camera_thread_right.join()

        self.lock_left.acquire()
        self.left_kill = False
        self.lock_left.release()

        self.lock_right.acquire()
        self.right_kill = False
        self.lock_right.release()

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

        self.destroy_threads()
        self.create_threads()

    def get_default_config(self):
        return CameraNodeConfig()
    
    def apply_transformations(self, frame, dir):
        if dir == "left":
            if self.config.left_flip_horizontal:
                frame = cv2.flip(frame, 1)
            if self.config.left_flip_vertical:
                frame = cv2.flip(frame, 0)
            if self.config.rotate_left_clockwise:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            else:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            if self.config.right_flip_horizontal:
                frame = cv2.flip(frame, 1)
            if self.config.right_flip_vertical:
                frame = cv2.flip(frame, 0)
            if self.config.rotate_right_clockwise:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            else:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        frame.resize(self.config.output_width, self.config.output_height)
        return frame

    def camera_worker(self, *args, **kwargs):
        index_name = args[0] if len(args) > 0 else ""
        camera_index = self.config.camera_index_left if index_name == "left" else self.config.camera_index_right

        capture = None
        while rclpy.ok() and self.system_state != SystemStateEnum.SHUTDOWN and (index_name == "left" and not self.left_kill or index_name == "right" and not self.right_kill):
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
                capture.set(cv2.CAP_PROP_AUTOFOCUS, self.autofocus)
                capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.autoexposure)
            except:
                time.sleep(self.config.scan_rate)
                continue

            while rclpy.ok() and self.system_state != SystemStateEnum.SHUTDOWN:
                try:
                    ret, frame = capture.read()
                    self.apply_transformations(frame, index_name)
                except:
                    if capture is  None:
                        break

                    capture.release()
                    capture = None
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
