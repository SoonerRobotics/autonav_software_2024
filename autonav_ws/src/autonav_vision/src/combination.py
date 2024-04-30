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
g_mapData.width = 100
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class ImageCombinerConfig:
    def __init__(self):
        self.map_res = 80
        self.top_left = [147, 287]
        self.top_right = [851, 232]
        self.bottom_right = [955, 678]
        self.bottom_left = [0, 678]

class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_image_combiner")

    def init(self):
        self.image_left = None
        self.image_right = None
        self.image_left_subscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/left", self.image_received_left, self.qos_profile)
        self.image_right_subscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/raw/image/right", self.image_received_right, self.qos_profile)
        self.combined_image_publisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/raw", self.qos_profile)
        self.combined_image_publisher_debug = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/debug", self.qos_profile)
        self.set_device_state(DeviceStateEnum.OPERATING)

    def image_received_left(self, msg):
        self.image_left = msg
        self.try_combine_images()

    def image_received_right(self, msg):
        self.image_right = msg
        self.try_combine_images()

    def order_points(self, pts):
        # initialzie a list of coordinates that will be ordered
        # such that the first entry in the list is the top-left,
        # the second entry is the top-right, the third is the
        # bottom-right, and the fourth is the bottom-left
        rect = np.zeros((4, 2), dtype = "float32")
        # the top-left point will have the smallest sum, whereas
        # the bottom-right point will have the largest sum
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        # now, compute the difference between the points, the
        # top-right point will have the smallest difference,
        # whereas the bottom-left will have the largest difference
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        # return the ordered coordinates
        return rect


    def four_point_transform(self, image, pts):
        # obtain a consistent order of the points and unpack them
        # individually
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect
        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")
        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        # return the warped image
        return warped

    def try_combine_images(self):
        if self.image_left is None or self.image_right is None:
            return
        
        # Combine the images to form (IMAGE_WIDTH * 2) x IMAGE_HEIGHT
        # The left image will be on the left, and the right image will be on the right
        cv2img_left = g_bridge.compressed_imgmsg_to_cv2(self.image_left)
        cv2img_right = g_bridge.compressed_imgmsg_to_cv2(self.image_right)
        combined = cv2.hconcat([cv2img_left, cv2img_right])
        # ordered as: top-left, top-right, bottom-right, bottom-left
        pts = [self.config.top_left, self.config.top_right, self.config.bottom_right, self.config.bottom_left]
        combined = self.four_point_transform(combined, np.array(pts))
        datamap = cv2.resize(combined, dsize=(self.config.map_res, self.config.map_res), interpolation=cv2.INTER_LINEAR) / 2
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
