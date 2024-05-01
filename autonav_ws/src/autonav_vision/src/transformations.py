#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import cv2
import numpy as np

from nav_msgs.msg import MapMetaData, OccupancyGrid
import rclpy.qos
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import json

from scr.node import Node
from scr.states import DeviceStateEnum

g_bridge = CvBridge()

g_mapData = MapMetaData()
g_mapData.width = 100
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
        self.map_res = 80

        # # Default to entire image
        # self.left_topleft = [0, 0]
        # self.left_topright = [480, 0]
        # self.left_bottomright = [480, 640]
        # self.left_bottomleft = [0, 640]
        
        # self.right_topleft = [0, 0]
        # self.right_topright = [480, 0]
        # self.right_bottomright = [480, 640]
        # self.right_bottomleft = [0, 640]

        # Default to entire image
        self.left_topleft = [0, 150]
        self.left_topright = [640, 150]
        self.left_bottomright = [640, 350]
        self.left_bottomleft = [0, 350]
        
        self.right_topleft = [0, 150]
        self.right_topright = [640, 150]
        self.right_bottomright = [640, 350]
        self.right_bottomleft = [0, 350]

        self.disable_blur = False
        self.disable_hsv = False
        self.disable_region_of_disinterest = False
        self.disable_perspective_transform = False


class ImageTransformer(Node):
    def __init__(self, dir = "left"):
        super().__init__("autonav_vision_transformer")
        self.dir = dir

    def directionify(self, topic):
        return topic + "/" + self.dir

    def init(self):
        self.cameraSubscriber = self.create_subscription(CompressedImage, self.directionify("/autonav/camera/compressed") , self.onImageReceived, self.qos_profile)
        self.rawMapPublisher = self.create_publisher(OccupancyGrid, self.directionify("/autonav/cfg_space/raw"), self.qos_profile)
        self.smallImagePublisher = self.create_publisher(CompressedImage, self.directionify("/autonav/cfg_space/raw/image") + "_small", self.qos_profile)

        self.set_device_state(DeviceStateEnum.OPERATING)

    def config_updated(self, jsonObject):
        self.config = json.loads(self.jdump(jsonObject), object_hook=lambda d: SimpleNamespace(**d))

    def get_default_config(self):
        return ImageTransformerConfig()

    def getBlur(self):
        blur = self.config.blur_weight
        blur = max(1, blur)
        return (blur, blur)

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

    def regionOfDisinterest(self, img, vertices):
        mask = np.ones_like(img) * 255
        cv2.fillPoly(mask, vertices, 0)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def publishOccupancyMap(self, img):
        datamap = cv2.resize(img, dsize=(self.config.map_res, self.config.map_res), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)
        self.rawMapPublisher.publish(msg)

        # Publish msg as a 80x80 image
        preview_image = cv2.resize(img, dsize=(80, 80), interpolation=cv2.INTER_LINEAR)
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        preview_msg.format = "jpeg"
        self.smallImagePublisher.publish(preview_msg)

    def onImageReceived(self, image = CompressedImage):
        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image)

        # Blur it up
        if not self.config.disable_blur:
            for _ in range(self.config.blur_iterations):
                cv_image = cv2.blur(cv_image, self.getBlur())

        # Apply filter and return a mask
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        if not self.config.disable_hsv:
            lower = (self.config.lower_hue, self.config.lower_sat, self.config.lower_val)
            upper = (self.config.upper_hue, self.config.upper_sat, self.config.upper_val)
            mask = cv2.inRange(img, lower, upper)
            mask = 255 - mask
        else:
            mask = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply region of disinterest and flattening
        if not self.config.disable_region_of_disinterest:
            height = img.shape[0]
            width = img.shape[1]
            if self.dir == "left":
                region_of_disinterest_vertices = [
                    (width, height),
                    (width, height / 1.8),
                    (0, height)
                ]
            else:
                region_of_disinterest_vertices = [
                    (0, height),
                    (0, height / 1.8),
                    (width, height)
                ]
            mask = self.regionOfDisinterest(mask, np.array([region_of_disinterest_vertices], np.int32))
            mask[mask < 250] = 0

        # Apply perspective transform
        if not self.config.disable_perspective_transform:
            pts = [self.config.left_topleft, self.config.left_topright, self.config.left_bottomright, self.config.left_bottomleft] if self.dir == "left" else [self.config.right_topleft, self.config.right_topright, self.config.right_bottomright, self.config.right_bottomleft]
            mask = self.four_point_transform(mask, np.array(pts))

        # Actually generate the map
        self.publishOccupancyMap(mask)


def main():
    rclpy.init()
    node_left = ImageTransformer(dir = "left")
    node_right = ImageTransformer(dir = "right")
    Node.run_nodes([node_left, node_right])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
