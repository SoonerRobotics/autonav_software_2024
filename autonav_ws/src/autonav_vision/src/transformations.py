#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import cv2
import numpy as np

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import json

from scr.node import Node
from scr.states import DeviceStateEnum

g_bridge = CvBridge()

g_mapData = MapMetaData()
g_mapData.width = 200
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0


class ImageTransformerConfig:
    def __init__(self):
        self.lower_hue = 0
        self.lower_sat = 0
        self.lower_val = 0
        self.upper_hue = 255
        self.upper_sat = 95
        self.upper_val = 210
        self.blur_weight = 5
        self.blur_iterations = 3
        self.rod_offset = 40
        self.map_res = 80


class ImageTransformer(Node):
    def __init__(self, dir = "left"):
        super().__init__("autonav_vision_transformer")
        self.dir = dir

    def directionify(self, topic):
        return topic + "/" + self.dir

    def init(self):
        self.cameraSubscriber = self.create_subscription(CompressedImage, self.directionify("/autonav/camera/compressed") , self.onImageReceived, 1)
        self.rawMapPublisher = self.create_publisher(OccupancyGrid, self.directionify("/autonav/cfg_space/preraw"), 1)
        self.filteredImagePublisher = self.create_publisher(CompressedImage, self.directionify("/autonav/cfg_space/raw/image"), 1)

        self.set_device_state(DeviceStateEnum.OPERATING)

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return ImageTransformerConfig()

    def getBlur(self):
        blur = self.config.blur_weight
        blur = max(1, blur)
        return (blur, blur)

    def regionOfDisinterest(self, img, vertices):
        mask = np.ones_like(img) * 255
        cv2.fillPoly(mask, vertices, 0)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def flattenImage(self, img):
        top_left = (int)(img.shape[1] * 0.18), (int)(img.shape[0])
        top_right = (int)(img.shape[1] - img.shape[1] * 0.18), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0

        src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
        dest_pts = np.float32([[0, 640], [480, 640], [0, 0], [480, 0]])

        matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)
        output = cv2.warpPerspective(img, matrix, (480, 640))
        return output

    def publishOccupancyMap(self, img):
        datamap = cv2.resize(img, dsize=(self.config.map_res, self.config.map_res),
                             interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)
        self.rawMapPublisher.publish(msg)

    def onImageReceived(self, image = CompressedImage):
        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image)

        # Blur it up
        for _ in range(self.config.blur_iterations):
            cv_image = cv2.blur(cv_image, self.getBlur())

        # Apply filter and return a mask
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = (self.config.lower_hue, self.config.lower_sat, self.config.lower_val)
        upper = (self.config.upper_hue, self.config.upper_sat, self.config.upper_val)
        mask = cv2.inRange(img, lower, upper)
        mask = 255 - mask

        # Apply region of disinterest and flattening
        height = img.shape[0]
        width = img.shape[1]
        if self.direction == "left":
            region_of_disinterest_vertices = [
                (width, height),
                (width, height - (height / 1.5)),
                (width / 2.3, height)
            ]
        else:
            region_of_disinterest_vertices = [
                (0, height),
                (0, height - (height / 1.5)),
                (width / 1.5, height)
            ]

        # Apply region of disinterest and flattening
        mask = self.regionOfDisinterest(mask, np.array([region_of_disinterest_vertices], np.int32))
        mask[mask < 250] = 0
        mask = self.flattenImage(mask)

        # Actually generate the map
        self.publishOccupancyMap(mask)

        # Preview the image
        preview_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        cv2.polylines(preview_image, np.array([region_of_disinterest_vertices], np.int32), True, (0, 255, 0), 2)
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        preview_msg.header = image.header
        preview_msg.format = "jpeg"

        # Publish the preview
        self.filteredImagePublisher.publish(preview_msg)


def main():
    rclpy.init()
    node_left = ImageTransformer(dir = "left")
    node_right = ImageTransformer(dir = "right")
    Node.run_nodes([node_left, node_right])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
