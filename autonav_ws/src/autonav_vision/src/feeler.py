import cv2
from math import cos, sin, atan, radians, degrees
import tkinter
from tkinter import filedialog

MAX_LENGTH = 100

# colors
WHITE = (255, 255, 255)

WIDTH = 800
HEIGHT = 800

#FIXME do we want this to be a class method of vector or feeler or whatever?
# convert x and y coordinates so that they are relative to the center of the image
def centerCoordinates(x, y):
    return (x + WIDTH//2), (y + HEIGHT//2)

class Vector:
    # so there's no confusion when creating a vector because we are mixing coordinate systems all over the place,
    # just make a 0 vector and then you're supposed to call either setXY or setPolar to actually change the values
    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0
        self.legnth = 0

    # def __init__(self, x, y):
    #     self.x = x
    #     self.y = y
    #     self.updatePolar()
    
    # def __init__(self, angle, length):
    #     self.angle = angle
    #     self.lenght = length
    #     self.updateCartesian()
    
    def getXY(self):
        return self.x, self.y
    
    def getPolar(self):
        return self.angle, self.length
    
    def setXY(self, x, y):
        self.x = x
        self.y = y
        self.updatePolar()
    
    def setPolar(self, angle, length):
        self.angle = angle
        self.length = length
        self.updateCartesian()
    
    def setLength(self, length):
        self.length = length
        self.updateCartesian()
    
    def __add__(self, other):
        return self.x + other.x, self.y + other.y
    
    def __sub__(self, other):
        return self.x + other.x, self.y + other.y
    
    # called when polar coords have been set and need to update the associated cartesian ones
    def updateCartesian(self):
        #SOH CAH TOA
        #sin(theta) = x / length
        self.x = self.length * cos(self.angle)
        self.y = self.length * sin(self.angle)
    
    # called for when cartesian coords are updated but need to update the polar ones
    def updatePolar(self):
        # good 'ol distance formula (assuming (0,0) is the origin for both polar and cartesian)
        self.length = sqrt(self.x**2 + self.y**2)
        self.angle = degrees(atan(self.y / self.x)) #TODO verify if this is correct

    # draw the feeler on the given image
    def draw(self, image):
        cv2.line(image, centerCoordinates(0, 0), centerCoordinates(self.x, self.y))

class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.vel = 0
        self.heading = 0

        self.feelers = []
        for angle in range(0, 360, 10):
            v = Vector()
            v.setPolar(angle, MAX_LENGTH)

            self.feelers.append(v)

        # start pointing straight
        self.heading_arrow = Vector(MAX_LENGTH, 0)
    
    def update(self):
        # reset our heading
        self.heading_arrow.setPolar(0, 0)

        for feeler in self.feelers:
            # make a vector from the end of the current vector if it was at max length to the end of the vector at its current length
            error = Vector(MAX_LENGTH, feeler.angle) - feeler

            # add this vector to main heading arrow
            self.heading_arrow += error
        
        #TODO I think there's something else we need to do?
    
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
    
    mask = threshold(image)

    for feeler in robot.feelers:
        feeler.update(mask)
        feeler.draw(image) #TODO do we want to output the mask or the image?
    
    robot.update()
    robot.draw(image)

    cv2.imshow("image", image)
    cv2.waitKey(0) #TODO do we want to make this match the 8 fps or something? or videoWrite and not bother with real-time output?

cv2.destroyAllWindows()