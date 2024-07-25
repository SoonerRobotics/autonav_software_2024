import cv2
import numpy as np
from math import cos, sin, atan, radians, degrees, sqrt, pi
import tkinter
from tkinter import filedialog

DEBUG = False

MAX_LENGTH = 300

# colors
WHITE = (255, 255, 255)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)

# image shape is 800x1600x3; 1600 because it's two 800x800 side by side because dual camera
WIDTH = 960
HEIGHT = 640

#FIXME I think there's a built-in for this
def sign(x):
    return -1 if x < 0 else 1

# verticies for region-of-disinterest
# order is top-left, top-right, bottom-right, bottom-left
VERTICIES = (
    (285, 303),
    (616, 303),
    (722, 638),
    (262, 638)
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

#FIXME do we want this to be a class method of vector or feeler or whatever?
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
        # from https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        # regular slope-intercept equation:
        # f(x) = y = mx + b
        # equation of a line in terms of x and y:
        # m = Δy/Δx
        # y = (Δy/Δx)x + b
        # (Δx)y = (Δy)x + (Δx)b
        # f(x, y) = 0 = (Δy)x - (Δx)y + (Δx)b
        # in our case, b = 0, and the start of the line is always (0, 0) so this simplifies to f(x, y) = 0 = (self.original_y * x)  -  (self.original_x * y)
        """plotLine(x0, y0, x1, y1)
            dx = x1 - x0
            dy = y1 - y0
            D = 2*dy - dx
            y = y0

            for x from x0 to x1
                plot(x, y)
                if D > 0
                    y = y + 1
                    D = D - 2*dx
                end if
                D = D + 2*dy"""
        # for our case x0 and y0 are always going to be 0 and 0 (i.e. the origin)
        y = 0
        x = 0

        y_dir = -1 if self.original_y < 0 else 1
        x_dir = -1 if self.original_x < 0 else 1

        try:
            # slope is rise / run, and starting point is always (0, 0) (i.e. y-intercept is 0)
            slope = self.original_y / self.original_x
        except ZeroDivisionError: # if x is 0, then it's a vertical line,
            slope = 2 # so it needs to be drawn by the y algorithm instead of the x one (i.e. the 2nd for loop)

        print(slope)

        # if the slope is less than 1 that means if we increment x, y will increase by either 0 or 1
        if abs(slope) <= 1:
            difference = (2 * self.original_y) - abs(self.original_x) #FIXME not sure if this needs to be absolutely valued or not

            # for each x in all the x coords we need to iterate through
            for x in range(0, self.original_x, x_dir):
               # get the coordinates for the image (because the origin of the image is in the top left corner but our coordinates have the origin at the center of the image)
                centered_x, centered_y = centerCoordinates(x, y)

                # if the pixel at that location is NOT empty space (ie it is an obstacle)
                if mask[centered_y, centered_x].any() > 0:
                    # then we've reached our new length, so update that
                    self.setXY(x, y)

                    return # and stop iterating so we don't break something

                # figure out if y needs to increase or stay the same
                if difference > 0:
                    y += y_dir
                    difference += 2 * (self.original_y - self.original_x) #TODO does this need to be abs()'ed?
                else:
                    difference += 2 * self.original_y

        # if it's larger than 1 then we need to do it the opposite: if we increment y, x will increase by either 0 or 1
        else:
            difference = (2 * self.original_x) - abs(self.original_y)

            # print(f"{centered_x}, {centered_y}")

            # for each y in all the y coords we need to iterate through
            for y in range(0, self.original_y, y_dir):
                # get the coordinates for the image (because the origin of the image is in the top left corner but our coordinates have the origin at the center of the image)
                centered_x, centered_y = centerCoordinates(x, y)
                
                # if the pixel at that location is NOT empty space (ie it is an obstacle)
                if mask[centered_y, centered_x].any() > 0:
                    # then we've reached our new length, so update that
                    self.setXY(x, y)

                    return # and stop iterating so we don't break something

                # figure out if x needs to increase or stay the same
                if difference > 0:
                    x += x_dir
                    difference += 2 * (self.original_x - self.original_y) #TODO does this need to be abs()'ed?
                else:
                    difference += 2 * self.original_x


        # if we've made it through the loop without encountering any obstacles, then bring us back up to original length
        self.setXY(self.original_x, self.original_y)

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
        # for angle in range(10, 360, 30):
        angle = 315
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
            # print(f"feeler length: {feeler.length} | feeler angle: {feeler.angle} | feeler x: {feeler.x} | feeler y: {feeler.y}")
            
            # make a vector, from the end of the current vector if it was at max length, to the end of the vector at its current length
            # in practice, because everything starts at (0, 0), just add 180 to the angle so it's pointing the opposite direction and set its length to the length of the error
            error_vec = Feeler(0, 0)
            # error = MAX_LENGTH - feeler.length
            # error_vec.setPolar((feeler.angle + 180) % 360, error * 2)

            # print(f"feeler: {feeler.angle} | error_vec: {error_vec.angle}")

            error_vec.color = RED
            # error_vec.draw(image)

            # add this vector to main heading arrow
            self.heading_arrow += error_vec
        
        #TODO I think there's something else we need to do?
    
    # draw the heading vector to the screen
    def draw(self, image):
        self.heading_arrow.color = GREEN
        self.heading_arrow.draw(image)


robot = Robot()

root = tkinter.Tk()
root.withdraw()

PATH = filedialog.askopenfilename()
# bg_img = cv2.imread(PATH)
video = cv2.VideoCapture(PATH)

done = False # while debugging don't need to do every frame, waste of battery power
frame = 0
while video.isOpened() and not done:
# while video.isOpened():
    ret, image = video.read()

    if not ret:
        break # the end of the video

    frame += 1

    if frame < 400:
        continue # skip the first 100 frames because it's just the robot sitting there
    # elif frame == 152:
    #     cv2.imwrite("frame.png", image)
    
    mask = threshold(image)
    # image = cv2.bitwise_and(mask, image)
    # mask = image

    # perform the lidar
    for feeler in robot.feelers:
        feeler.update(mask)

    # these are in a seperate loop to avoid drawing on the mask while the other feelers still need it blank to update themselves
    for feeler in robot.feelers:
        feeler.draw(mask)
        feeler.draw(image) # draw on both of them so it doesn't matter which output is actually displayed


    robot.update()
    robot.draw(image)

    # print(f"{robot.feelers[0].x:.2f}, {robot.feelers[0].y:.2f} | {robot.feelers[0].angle:.2f}, {robot.feelers[0].length:.2f}")


    cv2.imshow("image", image)
    # cv2.imshow("image", mask)
    cv2.waitKey(0) #TODO do we want to make this match the 8 fps or something? or videoWrite and not bother with real-time output?

    # done = True
    if frame > 401:
        done = True

video.release()
cv2.destroyAllWindows()