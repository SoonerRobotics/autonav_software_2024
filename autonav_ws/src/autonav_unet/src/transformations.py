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

# bridge to convert from compressed image messages to opencv masks and back
g_bridge = CvBridge()

# a bunch of local map stuff
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

        # Reload the unet model
        #TODO figure out a way to not use absolute paths so it can run on things that aren't my laptop
        self.model = tf.keras.models.load_model('/mnt/c/Users/isasq/Documents/GitHub/autonav_software_2024/autonav_ws/src/autonav_unet/src/results/SCRUNet_model.keras')


    def configure(self):
        # create the nodes
        self.cameraSubscriber = self.create_subscription(CompressedImage, "/autonav/camera/compressed", self.onImageReceived, 3)
        self.rawMapPublisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/raw", 1)
        self.filteredImagePublisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/image", 1)

        # once all node are online, we're running
        self.setDeviceState(DeviceStateEnum.OPERATING)

    # this is required to be here because of inheritance or something, though functionally it does nothing
    def transition(self, old, updated):
        return

    # flatten the image (convert it from 3d to 2d) so it's like we have a top-down view of the course in front of us
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

    # publish the occupancy map or something
    # idk what this does or how it does it I didn't write this code
    #TODO rewrite this code
    def publishOccupancyMap(self, img):
        datamap = cv2.resize(img, dsize=(MAP_RES, MAP_RES), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        msg = OccupancyGrid(info=g_mapData, data=flat)
        self.rawMapPublisher.publish(msg)

    # main image callback, takes compressed image message from camera publisher
    def onImageReceived(self, image_: CompressedImage):
        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image_)

        #======================================================
        # Histogram equalization of HSV value channel
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        image[:,:,2] = cv2.equalizeHist(image[:,:,2])
        image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)

        # rotate image 90 degrees for the model for some reason
        image = cv2.transpose(image) / 255.0

        # resize the image for the model
        image = cv2.resize(image, (256, 256))

        # actually run the CNN on the image
        mask_ = self.model.predict(np.array([image, image], dtype=np.float32))[0]
        # mask_ = tf.squeeze(self.model(tf.expand_dims(image, axis=0)), axis=0) #TODO get this line to work so we're not doing some weird [image, image][0] list stuff

        # rotate the image 90 degrees back to how it was
        mask = cv2.transpose(mask_ * 255.0)

        # scale the image back up
        mask = cv2.resize(mask, (640, 480)) #TODO find a better way to do this because when it scales the image up it introduces a lot of artefacts


        # Apply region of disinterest and flattening
        # I don't know how it does this I didn't write this code
        #TODO rewrite this code
        height = mask.shape[0]
        width = mask.shape[1]
        region_of_disinterest_vertices=[
            (0, height),
            (width / 2, height / 2 + 100),
            (width, height)
        ]
        mask = self.regionOfDisinterest(mask, np.array([region_of_disinterest_vertices], np.int32))

        # get our top-down view so we can make our map
        mask = self.flattenImage(mask)

        # publish the final processed image
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(mask)
        preview_msg.header = image_.header
        preview_msg.format = "jpeg"
        self.filteredImagePublisher.publish(preview_msg)

        # Actually generate the map
        self.publishOccupancyMap(mask)

    # filter out the robot from the image so we don't detect ourselves
    # I don't know how this code works I didn't write it
    #TODO rewrite this code    
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
