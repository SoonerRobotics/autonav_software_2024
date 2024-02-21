#!/usr/bin/env python3

from types import SimpleNamespace
import rclpy
import json

from scr.node import Node
from scr.states import DeviceStateEnum
from sensor_msgs.msg import CompressedImage


IMAGE_WIDTH = 480
IMAGE_HEIGHT = 640


class ImageCombinerConfig:
    def __init__(self):
        self.blah = 0


class ImageCombiner(Node):
    def __init__(self):
        super().__init__("autonav_image_combiner")

    def init(self):
        self.image_left = None
        self.image_right = None
        self.image_left_subscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/preraw/left", self.image_received_left, 10)
        self.image_right_subscriber = self.create_subscription(CompressedImage, "/autonav/cfg_space/preraw/right", self.image_received_right, 10)
        self.combined_image_publisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw", 10)
        self.set_device_state(DeviceStateEnum.OPERATING)

    def image_received_left(self, msg):
        self.image_left = msg
        self.try_combine_images()

    def image_received_right(self, msg):
        self.image_right = msg
        self.try_combine_images()

    def try_combine_images(self):
        if self.image_left is None or self.image_right is None:
            return
        
        # Combine the images to form (IMAGE_WIDTH * 2) x IMAGE_HEIGHT
        # The left image will be on the left, and the right image will be on the right
        combined_image = CompressedImage()
        combined_image.header.stamp = self.get_clock().now().to_msg()
        combined_image.format = "jpeg"
        combined_image.data = self.image_left.data + self.image_right.data
        self.image_left = None
        self.image_right = None
        self.combined_image_publisher.publish(combined_image)

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
