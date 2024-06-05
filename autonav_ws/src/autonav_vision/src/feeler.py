import cv2
from math import cos, sin, atan, radians, degrees
import tkinter
from tkinter import filedialog


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __add__(self, other):
        return self.x + other.x, self.y + other.y
    
    def __sub__(self, other):
        return self.x - other.x, self.y - other.y
    
    def __neg__(self):
        return -self.x, -self.y

MAX_LENGTH = 100

# colors
WHITE = (255, 255, 255)

WIDTH = 800
HEIGHT = 800

# standard distance formula
def dist(x, y):
    return (self.x ** 2  +  self.y ** 2) ** 0.5

# convert x and y coordinates so that they are relative to the center of the image
def centerCoordinates(x, y):
    return (x + WIDTH//2), (y + HEIGHT//2)

# convert polar coordiantes of the feelers (distance and angle) to x and y coordinates
def polarToXY(distance, angle):
    x = distance * cos(radians(angle))
    y = distance * sin(radians(angle))

    return x, y

#TODO is this supposed to work with centered- or left-upper-corner- origin coordinates?
def xyToPolar(x, y):
    length = dist(x, y)
    angle = degrees(atan(y / x))


class Feeler:
    def __init__(self, max_length, angle):
        self.max_length = length
        self.angle = angle
        self.length = max_length
        
    def update(self, image):
        length = MAX_LENGTH

        #TODO figure out how to pixel walk along the line or something
        #TODO need to do lerp (linear interpolation, which is like how they draw lines or something)
        for pixel in pixels:
            if pixel >= OFF_WHITE:
                x, y = pixel - (0, 0)

                self.length, self.angle = xyToPolar(x, y)

    # draw the feeler on the given image
    def draw(self, image):
        cv2.line(image, centerCoordinates(0, 0), centerCoordinates(*polarToXY(self.length, self.angle)))

class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.vel = 0
        self.heading = 0

        self.feelers = [Feeler(MAX_LENGTH, angle) for angle in range(0, 360, 10)]

        # start pionting straight
        self.heading_arrow = Feeler(MAX_LENGTH, 0)
    
    def update(self):
        # reset our heading
        self.heading_arrow.length = 0
        self.heading_arrow.angle = 0

        for feeler in self.feelers:
            # should be negative, so the addition works out right
            delta = feeler.length - feeler.max_length

            #TODO add this vector to main heading arrow
    
    def draw(self, image):
        pass #TODO


robot = Robot()

root = tkinter.Tk()
root.withdraw()

PATH = filedialog.askopenfilename()
# bg_img = cv2.imread(PATH)
video = cv2.VideoCapture(PATH)

while video.isOpened():
    ret, image = video.read()

    if not ret:
        break # the end of the video
    
    #TODO do we want to read in the camera feed, the thresholded stuff, or the astar stuff?
    # I'm thinking we do just the cameras, cause I don't think we want to use the same perspective transform parameters, or any at all
    
    for feeler in robot.feelers:
        feeler.update(image)
        feeler.draw(image)
    
    robot.update()
    robot.draw(image)

    cv2.imshow("image", image)
    cv2.waitKey(0)

cv2.destroyAllWindows()