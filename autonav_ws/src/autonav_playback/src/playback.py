#!/usr/bin/env python3

from types import SimpleNamespace
from autonav_msgs.msg import IMUData, GPSFeedback, MotorFeedback, MotorInput, Position, MotorControllerDebug
from scr.states import DeviceStateEnum, SystemStateEnum
from sensor_msgs.msg import CompressedImage
from scr_msgs.msg import SystemState, DeviceState
from cv_bridge import CvBridge
from scr.node import Node
from datetime import datetime
import shutil
import rclpy
import cv2
import os
import json
import subprocess


class ImageTransformerConfig:
    def __init__(self):
        self.record_imu = True
        self.record_gps = True
        self.record_position = True
        self.record_feedback = True
        self.record_motor_debug = True
        self.record_raw_cameras = True
        self.record_filtered_cameras = True
        self.record_astar = True
        self.record_autonomous = True
        self.record_manual = True
        self.frame_rate = 8


class PlaybackNode(Node):
    def __init__(self):
        super().__init__("autonav_playback")

        self.startTime = datetime.now().timestamp()
        self.file = None
        self.file_name = None
        self.home_dir = os.path.expanduser("~")
        self.camera_index = 0
        self.filtered_index = 0
        self.astar_index = 0
        self.bridge = CvBridge()
        self.config = self.get_default_config()
        self.image_raw_left = None
        self.image_raw_right = None
        self.image_filtered_left = None
        self.image_filtered_right = None

    def init(self):
        self.imu_subscriber = self.create_subscription(IMUData, "/autonav/imu", self.imu_callback, 20)
        self.gps_subscriber = self.create_subscription(GPSFeedback, "/autonav/gps", self.gps_callback, 20)
        self.feedback_subscriber = self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.feedback_callback, 20)
        self.position_subscriber = self.create_subscription(Position, "/autonav/position", self.position_callback, 20)
        self.controller_debug_subscriber = self.create_subscription(MotorControllerDebug, "/autonav/MotorControllerDebug", self.controller_debug_callback, 20)
        self.thresholded_subscriber_left = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/left_small", self.filtered_callback_left, self.qos_profile)
        self.thresholded_subscriber_right = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/right_small", self.filtered_callback_right, self.qos_profile)
        self.camera_subscriber_left = self.create_subscription(CompressedImage, "/autonav/camera/compressed/left", self.camera_callback_left, self.qos_profile)
        self.camera_subscriber_right = self.create_subscription(CompressedImage, "/autonav/camera/compressed/right", self.camera_callback_right, self.qos_profile)
        self.astar_subscriber = self.create_subscription(CompressedImage, "/autonav/debug/astar/image", self.astar_callback, self.qos_profile)

        self.set_device_state(DeviceStateEnum.OPERATING)

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))
        self.get_logger().info(f"Updated config: {self.jdump(jsonObject)}")

    def get_default_config(self):
        return ImageTransformerConfig()

    def timestamp(self):
        return datetime.now().timestamp()

    def create_file(self):
        time = datetime.now()
        timeFrmt = time.strftime("%Y-%m-%d_%H-%M-%S")
        stateFrmt = "autonomous" if self.system_state == SystemStateEnum.AUTONOMOUS else "manual"
        return f"{stateFrmt}_{timeFrmt}"

    def create_video(self, folder, name):
        IMAGES_PATH = os.path.join(self.home_dir, ".scr", "playback", self.file_name, "images", folder)
        SAVE_PATH = os.path.join(self.home_dir, ".scr", "playback", self.file_name)
        with open(os.devnull, "wb") as devnull:
            subprocess.call(["ffmpeg", "-r", f"{self.config.frame_rate}", "-i", f"{IMAGES_PATH}/%d.jpg", "-vcodec", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", "-y", f"{SAVE_PATH}/{name}.mp4"], stdout=devnull, stderr=devnull)

    def create_entry(self):
        self.file_name = self.create_file()
        self.startTime = datetime.now().timestamp()

        BASE_PATH = os.path.join(self.home_dir, ".scr", "playback", self.file_name)
        os.makedirs(BASE_PATH, exist_ok=True)
        os.makedirs(os.path.join(BASE_PATH, "images", "thresholded"), exist_ok=True)
        os.makedirs(os.path.join(BASE_PATH, "images", "astar"), exist_ok=True)
        os.makedirs(os.path.join(BASE_PATH, "images", "camera"), exist_ok=True)

        self.file = open(os.path.join(BASE_PATH, "log.csv"), "w")
        self.file.write("timestamp, type\n")

        self.filtered_index = 0
        self.astar_index = 0
        self.camera_index = 0

        self.get_logger().info(f"Recording playback data at {BASE_PATH}")

    def close_entry(self):
        if self.file is None:
            return

        self.file.close()
        self.file = None

        # Zip up the folder at $HOME/.scr/playback/{fileName} and then delete it
        BASE_PATH = os.path.join(self.home_dir, ".scr", "playback", self.file_name)
        self.create_video("thresholded", "thresholded")
        self.create_video("astar", "astar")
        self.create_video("camera", "camera")

        # For every type of log in the log file, create a seperate csv file for it
        with open(os.path.join(BASE_PATH, "log.csv"), "r") as logFile:
            for line in logFile.readlines()[1:]:
                logEntry = line.split(", ")
                logType = logEntry[1].strip()
                logPath = os.path.join(BASE_PATH, f"{logType}.csv")
                with open(logPath, "a") as logTypeFile:
                    logTypeFile.write(line)

        # Delete the images folder
        shutil.rmtree(os.path.join(BASE_PATH, "images"), ignore_errors=True)

        shutil.make_archive(BASE_PATH, "zip", BASE_PATH)
        SIZE_OF_ZIP = os.path.getsize(BASE_PATH + ".zip") / 1024 / 1024
        TIME_ELAPSED = datetime.now().timestamp() - self.startTime
        shutil.rmtree(BASE_PATH, ignore_errors=True)

        self.get_logger().info(f"Finished recording playback data at {BASE_PATH}. Size of zip: {SIZE_OF_ZIP:.2f} MB. Time elapsed: {TIME_ELAPSED:.2f} seconds")

    def write_file(self, msg: str):
        if self.file is None:
            return

        self.file.write(msg + "\n")

    def write_image(self, img: CompressedImage, relative_path: str, idx: int):
        if self.file_name is None:
            return

        IMAGE_PATH = os.path.join(self.home_dir, ".scr", "playback", self.file_name, "images", relative_path)
        cv2Image = self.bridge.compressed_imgmsg_to_cv2(img, "bgr8")
        cv2.imwrite(os.path.join(IMAGE_PATH, f"{idx}.jpg"), cv2Image)

    def system_state_transition(self, old: SystemState, updated: SystemState):
        self.get_logger().info(f"System state transitioned from {old.state} to {updated.state}")
        if old.state == SystemStateEnum.AUTONOMOUS and updated.state != SystemStateEnum.AUTONOMOUS:
            self.close_entry()

        if old.state == SystemStateEnum.MANUAL and updated.state != SystemStateEnum.MANUAL:
            self.close_entry()

        if old.state != SystemStateEnum.AUTONOMOUS and updated.state == SystemStateEnum.AUTONOMOUS and self.config.record_autonomous:
            self.create_entry()

        if old.state != SystemStateEnum.MANUAL and updated.state == SystemStateEnum.MANUAL and self.config.record_manual:
            self.create_entry()

    def imu_callback(self, msg: IMUData):
        if not self.config.record_imu:
            return

        self.write_file(f"{self.timestamp()}, ENTRY_IMU, {msg.accel_x}, {msg.accel_y}, {msg.accel_z}, {msg.angular_x}, {msg.angular_y}, {msg.angular_z}, {msg.roll}, {msg.pitch}, {msg.yaw}")

    def gps_callback(self, msg: GPSFeedback):
        if not self.config.record_gps:
            return

        self.write_file(f"{self.timestamp()}, ENTRY_GPS, {msg.latitude}, {msg.longitude}, {msg.altitude}, {msg.gps_fix}, {msg.is_locked}, {msg.satellites}")

    def feedback_callback(self, msg: MotorFeedback):
        if not self.config.record_feedback:
            return

        self.write_file(f"{self.timestamp()}, ENTRY_FEEDBACK, {msg.delta_x}, {msg.delta_y}, {msg.delta_theta}")

    def position_callback(self, msg: Position):
        if not self.config.record_position:
            return

        self.write_file(f"{self.timestamp()}, ENTRY_POSITION, {msg.x}, {msg.y}, {msg.theta}, {msg.latitude}, {msg.longitude}")

    def controller_debug_callback(self, msg: MotorControllerDebug):
        if not self.config.record_motor_debug:
            return

        self.write_file(f"{self.timestamp()}, ENTRY_MOTORDEBUG, {msg.current_forward_velocity}, {msg.forward_velocity_setpoint}, {msg.current_angular_velocity}, {msg.angular_velocity_setpoint}, {msg.left_motor_output}, {msg.right_motor_output}")

    def filtered_callback_left(self, msg: CompressedImage):
        if not self.config.record_filtered_cameras:
            return

        self.image_filtered_left = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        if self.image_filtered_right is not None:
            self.filtered_callback_combined()

    def filtered_callback_right(self, msg: CompressedImage):
        if not self.config.record_filtered_cameras:
            return
        
        self.image_filtered_right = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        if self.image_filtered_left is not None:
            self.filtered_callback_combined()

    def filtered_callback_combined(self):
        if not self.config.record_filtered_cameras:
            return

        # Scale the images to the 800 x 800
        self.image_filtered_left = cv2.resize(self.image_filtered_left, (800, 800))
        self.image_filtered_right = cv2.resize(self.image_filtered_right, (800, 800))

        img = cv2.hconcat([self.image_filtered_left, self.image_filtered_right])
        msg = self.bridge.cv2_to_compressed_imgmsg(img)
        self.write_file(f"{self.timestamp()}, ENTRY_FILTERED_IMAGE, /images/thresholded/{self.filtered_index}.jpg")
        self.write_image(msg, "thresholded", self.filtered_index)
        self.filtered_index += 1
        self.image_filtered_left = None
        self.image_filtered_right = None

    def astar_callback(self, msg: CompressedImage):
        if not self.config.record_astar:
            return
        
        # Scale the images to the 800 x 800
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        img = cv2.resize(img, (800, 800))
        msg = self.bridge.cv2_to_compressed_imgmsg(img)

        self.write_file(f"{self.timestamp()}, ENTRY_ASTAR_IMAGE, /images/expandified/{self.astar_index}.jpg")
        self.write_image(msg, "astar", self.astar_index)
        self.astar_index += 1

    def camera_callback_left(self, msg: CompressedImage):
        if not self.config.record_raw_cameras:
            return

        self.image_raw_left = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        if self.image_raw_right is not None:
            self.camera_callback_combined()

    def camera_callback_right(self, msg: CompressedImage):
        if not self.config.record_raw_cameras:
            return

        self.image_raw_right = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        if self.image_raw_left is not None:
            self.camera_callback_combined()

    def camera_callback_combined(self):
        if not self.config.record_raw_cameras:
            return

        img = cv2.hconcat([self.image_raw_left, self.image_raw_right])
        msg = self.bridge.cv2_to_compressed_imgmsg(img)  # Bad but works
        self.write_file(f"{self.timestamp()}, ENTRY_CAMERA_IMAGE, /images/camera/{self.camera_index}.jpg")
        self.write_image(msg, "camera", self.camera_index)
        self.camera_index += 1
        self.image_raw_left = None
        self.image_raw_right = None


def main():
    rclpy.init()
    node = PlaybackNode()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
