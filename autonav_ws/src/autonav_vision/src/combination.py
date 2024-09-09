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
        self.grid_left_subscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/left", self.grid_received_left, 1)
        self.grid_right_subscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/right", self.grid_received_right, 1)
        
        self.image_left = None
        self.image_right = None
        self.debug_camera_subscriber_left = self.create_subscription(CompressedImage, "/autonav/camera/compressed/left/pre_cutout", self.debug_image_received_left, self.qos_profile)
        self.debug_camera_subscriber_right = self.create_subscription(CompressedImage, "/autonav/camera/compressed/right/pre_cutout", self.debug_image_received_right, self.qos_profile)
        
        self.combined_grid_publisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/combined", 1)
        self.combined_grid_image_publisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/combined/image", self.qos_profile)
        self.combined_debug_camera_publisher = self.create_publisher(CompressedImage, "/autonav/camera/compressed/combined/pre_cutout", 1)
        self.set_device_state(DeviceStateEnum.OPERATING)

    def grid_received_left(self, msg):
        self.grid_left = g_bridge.compressed_imgmsg_to_cv2(msg)
        self.try_combine_grids()

    def grid_received_right(self, msg):
        self.grid_right = g_bridge.compressed_imgmsg_to_cv2(msg)
        self.try_combine_grids()

    def try_combine_grids(self):
        if self.grid_left is None or self.grid_right is None:
            return
        
        # try to combine the images now
        combined = np.concatenate((self.grid_left, self.grid_right), axis=1)

        self.combined_grid_publisher.publish(g_bridge.cv2_to_compressed_imgmsg(combined))

        #FIXME all this junk below
        # # Publish the combined grid as an image
        # preview_image = np.zeros((80, 80), dtype=np.uint8)
        # for i in range(80):
        #     for j in range(80):
        #         preview_image[i, j] = 0 if combined_grid.data[i * 80 + j] <= 10 else 255
        # preview_image = cv2.cvtColor(preview_image, cv2.COLOR_GRAY2RGB)
        # preview_image = cv2.resize(preview_image, (320, 320))

        # # Draw a grid on the image that is the scale of the original image, so it should be a 80x80 grid scaled up 4x
        # for i in range(80):
        #     cv2.line(preview_image, (0, i * 4), (320, i * 4), (85, 85, 85), 1)
        #     cv2.line(preview_image, (i * 4, 0), (i * 4, 320), (85, 85, 85), 1)

        # compressed_image = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        # self.combined_grid_image_publisher.publish(compressed_image)
    
    def debug_image_received_left(self, msg: CompressedImage):
        self.image_left = g_bridge.compressed_imgmsg_to_cv2(msg)
        self.try_combine_images()

    def debug_image_received_right(self, msg: CompressedImage):
        self.image_right = g_bridge.compressed_imgmsg_to_cv2(msg)
        self.try_combine_images()
    
    def try_combine_images(self):
        if self.image_left is None or self.image_right is None:
            return

        # try to combine the images now
        combined = np.concatenate((self.image_left, self.image_right), axis=1)

        self.combined_debug_camera_publisher.publish(g_bridge.cv2_to_compressed_imgmsg(combined))

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
