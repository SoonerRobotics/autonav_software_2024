#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from math import cos, sin, atan, radians, degrees, sqrt, pi
import json
from cv_bridge import CvBridge

from scr.node import Node
from scr.states import DeviceStateEnum

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from autonav_msgs.msg import Position, IMUData, PathingDebug, SafetyLights, MotorInput
from scr_msgs.msg import SystemState

from sensor_msgs.msg import CompressedImage
from autonav_msgs.msg import Position, MotorInput

def clamp(val, min_, max_):
    return max(min(val, max_), min_)

CV_BRIDGE = CvBridge()

MAX_LENGTH = 200

# colors
WHITE = (255, 255, 255)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)

# image shape is 800x1600x3; 1600 because it's two 800x800 side by side because dual camera
WIDTH = 960
HEIGHT = 640

# return the sign of the number
def sign(x):
    return -1 if x < 0 else 1

class Feeler:
    # create a 2 dimensional vector with cartesian coordinates
    # (0, 0) is assumed to be the center of the image, this will take some finangling to make work
    # length is auto-calculated and should never be manually set
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.length = self.dist(x, y)

        # necessary so the vector can grow back up to original size if there isn't an obstacle in the way
        self.original_x = x
        self.original_y = y

        self.color = BLUE
    
    # get x and y cartesian coordinates as a tuple, from (0,0) so not centered in the image
    def getXY(self):
        return self.x, self.y

    # get the vector as an angle and length FIXME
    def toPolar(self):
        try:
            angle = atan(self.y / self.x)
        except ZeroDivisionError:
            angle = 180

        return angle, self.length
    
    # set the x and y cartesian coordinates of the vector, other attributes will be updated accordingly
    def setXY(self, x, y):
        self.x = x
        self.y = y

        self.length = self.dist(x, y)
    
    # convert x and y coordinates so that they are relative to the center of the image
    def centerCoordinates(self, x, y):
        return (x + WIDTH//2), (y + HEIGHT//2 + 100)

    # draw the feeler on the given image
    def draw(self, image):
        startPt = self.centerCoordinates(0, 0)
        endPt = self.centerCoordinates(self.x, self.y)
        
        cv2.line(image, startPt, endPt, self.color, thickness=5)
    
    # mask is supposed to be a binary openCV image I think
    def update(self, mask):
        x = 0
        y = 0
        
        prev_x = 0
        prev_y = 0

        new_x = 0
        new_y = 0

        x_dir = sign(self.original_x)
        y_dir = sign(self.original_y)

        # for vertical lines, assign a slope of None and treat them as a special case in the main loop
        try:
            slope = self.original_y / self.original_x
        except ZeroDivisionError:
            slope = None

        while True:
            # vertical line, just need to move along the y-axis
            if slope is None:
                y += 1
            # horizontal line, just move along th x-axis
            elif slope == 0:
                x += 1

            # if slope is shallow, make x the independent variable
            elif abs(slope) <= 1:
                # get the y as a function of x
                new_y = abs(slope) * x

                # if the new y is higher than the previous one
                if (new_y - prev_y) > 0:
                    y += 1 # then go up by 1 y

                x += 1
                prev_y = y

            # slope is steep, do y as independent variable
            else:
                # get x as a function of y
                new_x = abs(1/slope) * y

                # and then if the new x is larger than the old one
                if (new_x - prev_x) > 0:
                    x += 1 # go up by one

                y += 1
                prev_x = x
            
            # if any of the pixel's color values (in RGB I think) are > 0 then
            check_x, check_y = self.centerCoordinates(x*x_dir, y*y_dir)
            if mask[check_y, check_x].any() > 0:
                # that is our new length
                self.setXY(x*x_dir, y*y_dir)
                return # and quit so we don't keep looping 'cause we found an obstacle
            
            elif abs(x) > abs(self.original_x) or abs(y) > abs(self.original_y): # if we're past our original farthest point
                self.setXY(self.original_x, self.original_y) # then we found no obstacle, and should stop looping
                return


    def __add__(self, other):
        ret = Feeler(self.x + other.x, self.y + other.y)
        ret.color = self.color
        
        return ret
    
    def __sub__(self, other):
        ret = Feeler(self.x - other.x, self.y - other.y)
        ret.color = self.color
        
        return ret

    # distance from (0, 0) to given coordinates
    def dist(self, x, y):
        return sqrt(x**2 + y**2)


# verticies for region-of-disinterest
# order is top-left, top-right, bottom-right, bottom-left
VERTICIES = (
    (285, 450),
    (616, 450),
    (722, 639),
    (262, 639)
)

# HSV thresholding values for obstacle detection
lower = (0, 0, 0)
upper = (255, 95, 210)

# kernel for erode/dilate
kernel = cv2.getStructuringElement(2, (2, 2))


class FeelerNode(Node):
    def __init__(self):
        super().__init__("autonav_feelers")
    
    def init(self):
        self.image_left_subscriber = self.create_subscription(CompressedImage, "/autonav/camera/compressed/left", self.on_left_image_received, self.qos_profile)
        self.image_right_subscriber = self.create_subscription(CompressedImage, "/autonav/camera/compressed/right", self.on_right_image_received, self.qos_profile)
        self.position_subscriber = self.create_subscription(Position, "/autonav/position", self.on_position_received, 1)

        self.camera_publisher_left = self.create_publisher(CompressedImage, "/autonav/camera/compressed/left/cutout", self.qos_profile)
        self.camera_publisher_right = self.create_publisher(CompressedImage, "/autonav/camera/compressed/right/cutout", self.qos_profile)
        self.filtered_image_left_publisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/image/left_small", self.qos_profile)
        self.filtered_image_right_publisher = self.create_publisher(CompressedImage, "/autonav/cfg_space/raw/image/right_small", self.qos_profile)
        self.motor_publisher = self.create_publisher(MotorInput, "/autonav/MotorInput", 1)

        self.x = 0
        self.y = 0
        self.heading = 0

        self.feelers = []
        # for angle in range(0, 359, 10):
        for angle in range(0, 359, 30):
            # given an angle, with a length of MAX_LENGTH (i.e. polar coordinates)
            # SOH CAH TOA
            x = round(MAX_LENGTH * cos(radians(angle)))
            y = round(MAX_LENGTH * sin(radians(angle)))
        
            self.feelers.append(Feeler(x, y))
        # self.feelers.append(Feeler(MAX_LENGTH, 0))
        # self.feelers.append(Feeler(-MAX_LENGTH, 0))
        # self.feelers.append(Feeler(0, MAX_LENGTH))
        # self.feelers.append(Feeler(0, -MAX_LENGTH))
        
        self.heading_arrow = Feeler(0, 0)
        self.heading_arrow.color = GREEN

        self.left_image = None
        self.right_image = None

        self.position = None

        self.set_device_state(DeviceStateEnum.OPERATING)
    
    def update(self):
        # reset our heading
        self.heading_arrow.setXY(0, 0)

        for feeler in self.feelers:
            # make a vector, from the end of the current vector if it was at max length, to the end of the vector at its current length
            # we can use simple vector subtraction because math
            original_feeler = Feeler(feeler.original_x, feeler.original_y)
            error_vec = feeler - original_feeler

            error_vec.color = RED
            # error_vec.draw(image)

            error_vec.length /= 2

            # add this vector to main heading arrow
            self.heading_arrow += error_vec

    #TODO document this
    def threshold(self, image):
        img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img, lower, upper)
        mask = 255 - mask

        mask = cv2.fillConvexPoly(mask, np.array(VERTICIES, dtype=np.int32), (0))

        mask = cv2.erode(mask, kernel)
        mask = cv2.dilate(mask, kernel)
        
        return mask

    # draw the heading vector to the screen
    def draw(self, image):
        self.heading_arrow.color = GREEN
        self.heading_arrow.draw(image)

    def on_left_image_received(self, image: CompressedImage):
        # Decompress image
        img = CV_BRIDGE.compressed_imgmsg_to_cv2(image)

        self.left_image = img

        self.camera_publisher_left.publish(image)

    def on_right_image_received(self, image: CompressedImage):
        # Decompress image
        img = CV_BRIDGE.compressed_imgmsg_to_cv2(image)

        self.right_image = img

        if self.left_image is None or self.right_image is None:
            return

        # try to combine the images now
        combined = np.concatenate((self.left_image, self.right_image), axis=1)

        #TODO flatten image

        mask = self.threshold(combined)

        # split the big image into two images again to publish them (copy/pasted from unet_model_copy/split_images_in_half.py)
        self.left_filtered_image = mask[:, :WIDTH//2]
        self.right_filtered_image = mask[:, WIDTH//2:]

        # perform the lidar
        for feeler in self.feelers:
            feeler.update(mask)
        
        # these are in a seperate loop to avoid drawing on the mask while the other feelers still need it blank to update themselves
        for feeler in self.feelers:
            feeler.draw(mask)
            feeler.draw(img) # draw on both of them so it doesn't matter which output is actually displayed
        
        self.update()
        self.draw(img)

        self.filtered_image_left_publisher.publish(CV_BRIDGE.cv2_to_compressed_imgmsg(self.left_filtered_image))
        self.filtered_image_right_publisher.publish(CV_BRIDGE.cv2_to_compressed_imgmsg(self.right_filtered_image))

        inputPacket = MotorInput()
        angle, speed = self.heading_arrow.toPolar()

        # print(f"speed: {speed} | angle: {angle}")
        # self.get_logger().info(f"speed: {speed} | angle: {angle}")

        # clamp speed temporarily FIXME
        speed = clamp(speed, -1, 1)

        # print(f"post-clamp speed: {speed} | angle: {angle}")
        # self.get_logger().info(f"post-clamp speed: {speed} | angle: {angle}")

        # not sure if we need the getAngleDifference() from astar.py or not
        angle_difference = (angle - self.position.theta) % 2*pi

        inputPacket.forward_velocity = float(speed) # it is very important that this is a float
        inputPacket.angular_velocity = angle_difference*0.5

        inputPacket.forward_velocity = float(0.5)

        # if degrees(self.heading_arrow.toPolar()[0]) > 10:
        if self.heading_arrow.toPolar()[0] > 0.1:
            # go left
            inputPacket.angular_velocity = float(0.05)
        # elif degrees(self.heading_arrow.toPolar()[0]) < -10:
        elif self.heading_arrow.toPolar()[0] < 0.1:
            # go right
            inputPacket.angular_velocity = float(-0.05)
        else:
            inputPacket.angular_velocity = float(0)
        
        # inputPacket.angular_velocity = float(0)

        self.motor_publisher.publish(inputPacket)

        self.camera_publisher_right.publish(image)

    def on_position_received(self, msg):
        self.position = msg


def main():
    rclpy.init()
    node = FeelerNode()
    Node.run_node(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
