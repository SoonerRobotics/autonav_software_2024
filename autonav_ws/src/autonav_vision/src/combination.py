#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import json
import cv2
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose

from scr.node import Node
from scr.states import DeviceStateEnum
from nav_msgs.msg import OccupancyGrid


g_bridge = CvBridge()
g_mapData = MapMetaData()
g_mapData.width = 100
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_vision_combiner")

    def init(self):
        self.grid_left = None
        self.grid_right = None
        self.grid_left_subscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/raw/left", self.grid_received_left, self.qos_profile)
        self.grid_right_subscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/raw/right", self.grid_received_right, self.qos_profile)
        self.combined_grid_publisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/combined", self.qos_profile)
        self.combined_grid_image_publisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/combined/image", self.qos_profile)
        self.set_device_state(DeviceStateEnum.OPERATING)

    def grid_received_left(self, msg):
        self.grid_left = msg
        self.try_combine_grids()

    def grid_received_right(self, msg):
        self.grid_right = msg
        self.try_combine_grids()

    def try_combine_grids(self):
        if self.grid_left is None or self.grid_right is None:
            return
        
        combined_grid = OccupancyGrid()
        combined_grid.header = self.grid_left.header
        combined_grid.info = self.grid_left.info
        combined_grid.data = [0] * len(self.grid_left.data)
        for i in range(len(self.grid_left.data)):
            if self.grid_left.data[i] == 127 or self.grid_right.data[i] == 127:
                combined_grid.data[i] = 127
            else:
                combined_grid.data[i] = 0

        # Publish the combined grid
        self.grid_left = None
        self.grid_right = None
        self.combined_grid_publisher.publish(combined_grid)

        # publish debug image, 127 = white, 0 = black
        preview_image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), np.uint8)
        for i in range(len(combined_grid.data)):
            x = i % combined_grid.info.width
            y = i // combined_grid.info.width
            if combined_grid.data[i] == 127:
                preview_image[y, x] = [255, 255, 255]
            else:
                preview_image[y, x] = [0, 0, 0]

        compressed_image = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        self.combined_grid_image_publisher.publish(compressed_image)

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return {}


def main():
    rclpy.init()
    node = ImageCombiner()
    Node.run_node(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
