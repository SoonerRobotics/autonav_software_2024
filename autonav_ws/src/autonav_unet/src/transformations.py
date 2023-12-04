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

# image flattening constants calculations
#TODO at some point we might not want these to be constant, but for now there's no need for them to be calculated every loop iteration/clutter the node class code
offset = 270

#TEMP FIXME
# top_left = (0+offset, 480)
# top_right = (640-offset, 480)
top_left = (0, 480)
top_right = (640, 480)
bottom_left = (0, 0)
bottom_right = (640, 0)

# source points are just the four corners of the image
src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
# dest_pts = np.float32([[0, 480],  [640, 480],  [0+offset, 0],      [640-offset, 0]])
dest_pts = np.float32([[0, 480],  [640, 480],  [0, 0],      [640, 0]])

# this is the actual thing we need to perform the flattening
MATRIX = cv2.getPerspectiveTransform(dest_pts, src_pts)

# and here's the constants for region of disinterest
# vertices according to MS paint and a frame I grabbed from Scrabby (now with some tuning from running in Scrabby)
region_of_disinterest_vertices = [(10, 479), (319, 390), (630, 479)] # triangle shape | (x, y) | bottom left, top, bottom right I think
region_of_disinterest = np.array([region_of_disinterest_vertices], np.int32)


# for erode/dilate/blur operations
# 0 for rectangle, 1 for cross, 2 for ellipse
EROSION_SHAPE = 2
KERNEL_SIZE = 3

# the actual kernel creating using OpenCV magic
kernel = cv2.getStructuringElement(EROSION_SHAPE, (KERNEL_SIZE, KERNEL_SIZE))
 

class ImageTransformer(Node):
    def __init__(self):
        super().__init__("autonav_vision_transformer")

        # Reload the unet model
        self.model = tf.keras.models.load_model('autonav_ws/src/autonav_unet/src/results/SCRUNet_model.keras')

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
        return cv2.warpPerspective(img, MATRIX, (640, 480)) # just let OpenCV work its magic.

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

        #=====================CNN THINGS=====================
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
        #=====================/CNN THINGS=====================

        # Apply region of disinterest and flattening
        # I don't really know how it does this but it does it
        cv_image = self.regionOfDisinterest(cv_image, region_of_disinterest)
        
        # erode the mask a little bit to remove artefacts and small bits that aren't actually there
        mask = cv2.erode(mask, kernel)

        # dilate it back up to make up for lost information
        mask = cv2.dilate(mask, kernel)


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
        # create a binary image from the given image? not sure what this line of code does
        mask = np.ones_like(img) * 255

        # have OpenCV work its magic and draw a triangle (the given vertices) on the mask
        cv2.fillPoly(mask, vertices, 0)

        # apply the mask (not keeping the pixels we don't want, keeping the ones we do)
        masked_image = cv2.bitwise_and(img, mask)

        return masked_image


def main():
    rclpy.init()
    rclpy.spin(ImageTransformer())
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
