import rclpy

import numpy as np
import cv2 as cv

from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage

bridge = CvBridge()


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.camera_publisher = self.create_publisher(CompressedImage, "/camera/compressed", 1)

        self.runCamera()

    def runCamera(self):
        capture = cv2.VideoCapture(0)
        
        while 1:
            ret, frame = capture.read()

            self.camera_publisher.publish(bridge.cv2_to_compressed_imgmsg(frame))
    
def main():
    rclpy.init()

    cameraNode = CameraNode()

    rclpy.spin(cameraNode)

    cameraNode.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()