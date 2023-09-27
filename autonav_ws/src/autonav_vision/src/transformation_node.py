import rclpy

import numpy as np
import cv2 as cv

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage

# Bridge for converting ROS image stuff to OpenCV image stuff and back
bridge = CvBridge()


# A node to handle all the thresholding and computer vision tasks for the robot
class TransformationNode(Node):
    def __init__(self):
        super().__init__("vision_transformation_node")

        # subscriber for the CompressedImage topic
        self.camera_subscriber = self.create_subscription(CompressedImage, "/camera/compressed", self.onImage, 10)

        # publisher for the binary output image
        self.filtered_publisher = self.create_publisher(CompressedImage, "/autonav/camera/filtered", 1)

        # Threshold HSV values
        self.lower = (0, 0, 0) #TODO
        self.upper = (255, 255, 255) #TODO
    
    # HSV thresholding for detecting obstacles/lines
    def thresholdImage(self, image):
        # apply the HSV threshold
        mask = cv.inRange(image, self.lower, self.upper)

        # invert mask (because we threshold for the ground, and anything that's not the ground is an obstacle)
        # which might change
        mask = 255 - mask
        mask[mask < 250] = 0

    # /camera/compressed subscriber callback
    def onImage(self, data):
        # turn image into something we can work with
        image = bridge.compressed_imgmsg_to_cv2(data)
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # blur image (except maybe not)
        #TODO

        # threshold image
        self.thresholdImage(image)

        # filter out the robot bits that are in-frame
        #TODO

        self.detectObstacles(data)


        # turn the image back into something that ROS can send
        image = bridge.cv2_to_compressed_imgmsg(image)

        # and publish it
        self.filtered_publisher.publish(image)
    
def main():
    rclpy.init()

    transformationNode = TransformationNode()

    rclpy.spin(transformationNode)

    transformationNode.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()