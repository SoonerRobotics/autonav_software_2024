#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import json
from cv_bridge import CvBridge
from nav_msgs.msg import MapMetaData, OccupancyGrid
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
            if self.grid_left.data[i] > 0 or self.grid_right.data[i] > 0:
                # Need to figure out what the value actually is: 255, 100, 1?
                combined_grid.data[i] = self.grid_left.data[i] if self.grid_left.data[i] > self.grid_right.data[i] else self.grid_right.data[i]
            else:
                combined_grid.data[i] = 0

        self.grid_left = None
        self.grid_right = None

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
