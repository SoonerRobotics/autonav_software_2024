#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import json
import cv2
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose

from scr.node import Node
from scr.states import DeviceStateEnum
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid


g_bridge = CvBridge()
g_mapData = MapMetaData()
g_mapData.width = 200
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class ImageCombinerConfig:
    def __init__(self):
        self.overlap = 0
        self.map_res = 80


class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_image_combiner")

    def init(self):
        self.image_left = None
        self.image_right = None
        self.image_left_subscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/left", self.image_received_left, 10)
        self.image_right_subscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/right", self.image_received_right, 10)
        self.combined_image_publisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/raw", 10)
        self.combined_image_publisher_debug = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/debug", 10)
        self.set_device_state(DeviceStateEnum.OPERATING)

    def image_received_left(self, msg):
        self.image_left = msg
        self.try_combine_images()

    def image_received_right(self, msg):
        self.image_right = msg
        self.try_combine_images()

    def try_combine_images(self):
        if self.image_left is None or self.image_right is None:
            return
        
        # Combine the images to form (IMAGE_WIDTH * 2) x IMAGE_HEIGHT
        # The left image will be on the left, and the right image will be on the right
        cv2img_left = g_bridge.compressed_imgmsg_to_cv2(self.image_left)
        cv2img_right = g_bridge.compressed_imgmsg_to_cv2(self.image_right)
        combined = cv2.hconcat([cv2img_left, cv2img_right])
        datamap = cv2.resize(combined, dsize=(self.config.map_res * 2, self.config.map_res), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)

        self.image_left = None
        self.image_right = None
        self.combined_image_publisher.publish(msg)
        compressed_msg = g_bridge.cv2_to_compressed_imgmsg(combined)
        self.combined_image_publisher_debug.publish(compressed_msg)

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return ImageCombinerConfig()


def main():
    rclpy.init()
    node = ImageCombiner()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
