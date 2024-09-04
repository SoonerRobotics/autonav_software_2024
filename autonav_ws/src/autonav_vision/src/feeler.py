import cv2
import numpy as np
from math import cos, sin, atan, radians, degrees, sqrt, pi
import tkinter
from tkinter import filedialog

MAX_LENGTH = 300

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

# verticies for region-of-disinterest
# order is top-left, top-right, bottom-right, bottom-left
VERTICIES = (
    (285, 303),
    (616, 303),
    (722, 500),
    (262, 500)
)

# HSV thresholding values for obstacle detection
lower = (0, 0, 0)
upper = (255, 95, 210)

# kernel for erode/dilate
kernel = cv2.getStructuringElement(2, (2, 2))

def threshold(image):
    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img, lower, upper)
    mask = 255 - mask

    mask = cv2.fillConvexPoly(mask, np.array(VERTICIES, dtype=np.int32), (0))

    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    
    return mask

# convert x and y coordinates so that they are relative to the center of the image
def centerCoordinates(x, y):
    return (x + WIDTH//2), (y + HEIGHT//2)

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
    
    # set the x and y cartesian coordinates of the vector, other attributes will be updated accordingly
    def setXY(self, x, y):
        self.x = x
        self.y = y

        self.length = self.dist(x, y)
    
    # draw the feeler on the given image
    def draw(self, image):
        startPt = centerCoordinates(0, 0)
        endPt = centerCoordinates(self.x, self.y)
        
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
            if mask[*centerCoordinates(x*x_dir, y*y_dir)[::-1]].any() > 0:
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



class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.heading = 0

        self.feelers = []
        for angle in range(0, 359, 10):
            # given an angle, with a length of MAX_LENGTH (i.e. polar coordinates)
            # SOH CAH TOA
            x = round(MAX_LENGTH * cos(radians(angle)))
            y = round(MAX_LENGTH * sin(radians(angle)))
        
            self.feelers.append(Feeler(x, y))
        

        # start pointing straight
        self.heading_arrow = Feeler(0, MAX_LENGTH)
        self.heading_arrow.color = GREEN
    
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

    # draw the heading vector to the screen
    def draw(self, image):
        self.heading_arrow.color = GREEN
        self.heading_arrow.draw(image)


robot = Robot()

root = tkinter.Tk()
root.withdraw()

PATH = filedialog.askopenfilename()
video = cv2.VideoCapture(PATH)
# videoOut = cv2.VideoWriter("./camera.mp4", cv2.VideoWriter.fourcc(*"mp4v"), 8.0, (960, 640))

done = False # while debugging don't need to do every frame, waste of battery power
frame = 0
while video.isOpened() and not done:
    ret, image = video.read()

    if not ret:
        break # the end of the video

    frame += 1

    if frame < 500:
        continue
    
    mask = threshold(image)
    # image = cv2.bitwise_and(mask, image)

    # perform the lidar
    for feeler in robot.feelers:
        feeler.update(mask)

    # these are in a seperate loop to avoid drawing on the mask while the other feelers still need it blank to update themselves
    for feeler in robot.feelers:
        feeler.draw(mask)
        feeler.draw(image) # draw on both of them so it doesn't matter which output is actually displayed


    robot.update()
    robot.draw(image)

    cv2.imshow("image", image)
    # cv2.imshow("image", mask)
    cv2.waitKey(0)

    # videoOut.write(image)

    # done = True
    if frame > 550:
        done = True

video.release()
# videoOut.release()
cv2.destroyAllWindows()