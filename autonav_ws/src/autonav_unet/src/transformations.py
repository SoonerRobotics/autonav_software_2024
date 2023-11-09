#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

import time

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

from scr_core.node import Node
from scr_core.state import DeviceStateEnum

import tensorflow as tf
from tensorflow import keras

g_bridge = CvBridge()

g_mapData = MapMetaData()
g_mapData.width = 200
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0

MAP_RES = 80


class ImageTransformer(Node):
    def __init__(self):
        super().__init__("autonav_vision_transformer")

        # Reload the model
        self.model = tf.keras.models.load_model('/mnt/c/Users/isasq/Documents/GitHub/autonav_software_2023/autonav_ws/src/autonav_unet/src/results/SCRUNet_model.keras')


    def configure(self):
        self.cameraSubscriber = self.create_subscription(CompressedImage, "/autonav/camera/compressed", self.onImageReceived, 5)
        self.rawMapPublisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/raw", 1)
        self.filteredImagePublisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/image", 1)

        self.setDeviceState(DeviceStateEnum.OPERATING)

    # this is required to be here because of inheritance or something, though functionally it does nothing
    def transition(self, old, updated):
        return

    def flattenImage(self, img):
        top_left = (int)(img.shape[1] * 0.26), (int)(img.shape[0])
        top_right = (int)(img.shape[1] - img.shape[1] * 0.26), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0

        src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
        dest_pts = np.float32([ [0, 480], [640, 480] ,[0, 0], [640, 0]])

        matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)
        output = cv2.warpPerspective(img, matrix, (640, 480))
        return output

    def publishOccupancyMap(self, img):
        datamap = cv2.resize(img, dsize=(MAP_RES, MAP_RES), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)
        self.rawMapPublisher.publish(msg)

    def onImageReceived(self, image_: CompressedImage):
        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image_)
        # cv2.imshow("input", cv_image)

        #======================================================
        # Histogram equalization of HSV value channel
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        image[:,:,2] = cv2.equalizeHist(image[:,:,2])
        image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)

        image = cv2.transpose(image) / 255.0
        image = cv2.resize(image, (256, 256))

        # cv2.imshow("image", image)
        # cv2.waitKey()
        # cv2.destroyAllWindows()

        start_time = time.time()
        mask_ = self.model.predict(np.array([image, image], dtype=np.float32))[0]
        # mask_ = tf.squeeze(self.model(tf.expand_dims(image, axis=0)), axis=0)
        # self.get_logger().error(time.time() - start_time)

        # print()
        # print(mask_)
        # print()

        # self.get_logger().error(mask_.shape)
        # cv2.imshow("mask", mask_)

        mask = cv2.transpose(mask_ * 255.0)
        mask = cv2.resize(mask, (640, 480))
        # cv2.imshow("image", mask)

        # cv2.imshow("mask2", mask)
        # cv2.waitKey()

        #==========================================================

        # Apply region of disinterest and flattening
        height = mask.shape[0]
        width = mask.shape[1]
        region_of_disinterest_vertices=[
            (0, height),
            (width / 2, height / 2 + 100),
            (width, height)
        ]
        mask = self.regionOfDisinterest(mask, np.array([region_of_disinterest_vertices], np.int32))
        # cv2.imshow("disinterest", mask)

        mask = self.flattenImage(mask)
        # cv2.imshow("flattened", mask)

        # cv2.waitKey()
        # preview_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(mask)
        # image = g_bridge.compressed_imgmsg_to_cv2(preview_msg)
        # cv2.imshow("processed", image)
        # cv2.waitKey()
        preview_msg.header = image_.header
        preview_msg.format = "jpeg"
        self.filteredImagePublisher.publish(preview_msg)

        # Actually generate the map
        self.publishOccupancyMap(mask)
    
    def regionOfDisinterest(self, img, vertices):
        mask = np.ones_like(img) * 255
        cv2.fillPoly(mask, vertices, 0)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image


def main():
    rclpy.init()
    rclpy.spin(ImageTransformer())
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
