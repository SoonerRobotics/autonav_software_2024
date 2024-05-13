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

    def scale_grid(self, grid):
        # Initialize a new grid of the same height but half the width
        scaled_grid = OccupancyGrid()
        scaled_grid.info = grid.info
        scaled_grid.info.width = 40
        scaled_grid.data = [0] * (80 * 80)
        
        # Copy every second column from the original grid to the new grid
        for y in range(80):
            for x in range(40):
                # Take every second element from the row
                scaled_grid.data[y * 40 + x] = grid.data[y * 80 + (2 * x)]
        
        return scaled_grid

    def try_combine_grids(self):
        if self.grid_left is None or self.grid_right is None:
            return

        # Scale down grids
        scaled_left = self.scale_grid(self.grid_left)
        scaled_right = self.scale_grid(self.grid_right)
        
        # Create a new 80x80 grid for the combined result
        combined_grid = OccupancyGrid()
        combined_grid.info = g_mapData
        combined_grid.info.width = 80
        combined_grid.info.height = 80
        combined_grid.data = [0] * (80 * 80)
        
        # Place each grid in the combined grid
        for y in range(80):
            for x in range(40):  # Fill from left scaled grid
                combined_grid.data[y * 80 + x] = scaled_left.data[y * 40 + x]
            for x in range(40):  # Fill from right scaled grid
                combined_grid.data[y * 80 + (40 + x)] = scaled_right.data[y * 40 + x]

        # Publish the combined grid
        self.grid_left = None
        self.grid_right = None
        self.combined_grid_publisher.publish(combined_grid)

        # Publish the combined grid as an image
        preview_image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH), np.uint8)
        for i in range(len(combined_grid.data)):
            x = i % combined_grid.info.width
            y = i // combined_grid.info.width
            if combined_grid.data[i] > 0:
                preview_image[y, x] = 255
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
