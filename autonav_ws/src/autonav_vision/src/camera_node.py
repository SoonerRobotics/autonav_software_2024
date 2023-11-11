import rclpy

import numpy as np
import cv2 as cv

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage

# Bridge for converting ROS image stuff to OpenCV image stuff and back
bridge = CvBridge()


# A node to publish CompressedImage messages to the /camera/compressed topic for use by the computer vision node
class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # publisher for the /camera/compressed topic
        self.camera_publisher = self.create_publisher(CompressedImage, "/camera/compressed", 1)

        # start publishing images
        self.runCamera()

    def runCamera(self):
        # initialize the camera
        capture = cv2.VideoCapture(0)

        # just run until we're killed        
        while 1:
            # no idea what ret is, but frame is the OpenCV image read from the camera stream
            ret, frame = capture.read()

            # publish it as a CompressedImage to the topic
            self.camera_publisher.publish(bridge.cv2_to_compressed_imgmsg(frame))
    
def main():
    rclpy.init()

    cameraNode = CameraNode()

    rclpy.spin(cameraNode)

    cameraNode.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()