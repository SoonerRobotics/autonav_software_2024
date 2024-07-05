import cv2
import numpy as np
from math import cos, sin, atan, radians, degrees, sqrt
import tkinter
from tkinter import filedialog

MAX_LENGTH = 175

# colors
WHITE = (255, 255, 255)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)

# image shape is 800x1600x3; 1600 because it's two 800x800 side by side because dual camera
WIDTH = 960
HEIGHT = 640

def threshold(image):
    # order is top-left, top-right, bottom-right, bottom-left
    vertices = (
        (285, 303),
        (616, 303),
        (722, 638),
        (262, 638)
    )

    # print(vertices)

    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = (0, 0, 0)
    upper = (255, 95, 210)
    mask = cv2.inRange(img, lower, upper)
    mask = 255 - mask

    mask = cv2.fillConvexPoly(mask, np.array(vertices, dtype=np.int32), (0))
    
    return mask

# === copypastad from transformations.py ===
def regionOfDisinterest(img, vertices):
    mask = np.ones_like(img) * 255
    cv2.fillPoly(mask, vertices, 0)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
# === /copypasta ===


#TODO find original implementation
def frange(start, stop, step=1):
    i = start
    while abs(i + step) < abs(stop): # absolute value here is very important, because we're passing in negative numbers
        i += step
        yield i
    yield stop

#FIXME do we want this to be a class method of vector or feeler or whatever?
# convert x and y coordinates so that they are relative to the center of the image
def centerCoordinates(x, y):
    return (x + WIDTH//2), (y + HEIGHT//2)

# taken from the wikipedia page on linear interpolation
def lerp(end):
    #FIXME play with these values?
    # but anyways yeah just assume all polar/cartesian rubbish starts at 0 or something
    start = 0
    step = 0.5
    # this is giving me major frange() vibes, which we honestly might need
    for x in range(start, round(end)):
        yield round(start + (step * (end - start)))
    # yield end # just in case?

class Vector:
    # so there's no confusion when creating a vector because we are mixing coordinate systems all over the place,
    # just make a 0 vector and then you're supposed to call either setXY or setPolar to actually change the values
    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0
        self.legnth = 0

        self.color = BLUE

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
        ret = Vector()
        ret.setXY(self.x + other.x, self.y + other.y)
        
        return ret
    
    def __sub__(self, other):
        ret = Vector()
        ret.setXY(self.x - other.x, self.y - other.y)
        
        return ret
    
    # called when polar coords have been set and need to update the associated cartesian ones
    def updateCartesian(self):
        #SOH CAH TOA
        #sin(theta) = x / length
        self.x = self.length * cos(self.angle)
        self.y = self.length * sin(self.angle)
    
    # called for when cartesian coords are updated but need to update the polar ones
    def updatePolar(self):
        try: 
            # good 'ol distance formula (assuming (0,0) is the origin for both polar and cartesian)
            self.length = sqrt(self.x**2 + self.y**2)
            self.angle = degrees(atan(self.y / self.x)) #TODO verify if this is correct
        except ZeroDivisionError:
            # if self.x is 0, then atan(y / x) will error because divide by 0, but that just means degrees should be 0 (or 360, not sure)
            self.angle = 0 #FIXME 360 instead?

    # draw the feeler on the given image
    def draw(self, image):
        startPt = centerCoordinates(0, 0)
        endPt = centerCoordinates(self.x, self.y)

        endPt = round(endPt[0]), round(endPt[1])

        cv2.line(image, startPt, endPt, self.color, thickness=5)
    
    # mask is supposed to be a binary openCV image I think
    def update(self, mask):
        # centeredX, centeredY = centerCoordinates(self.x, self.y)

        # max # of pixels is MAX_LENGTH, right? so step should be MAX_LENGTH / x_length, and x_length is just x
        # which means we need frange
        stepVal = MAX_LENGTH / self.x #TODO what if this is 0
        slope = self.y / self.x # rise over run, and we're assuming everything starts at (0, 0)

        # print(f"({self.x}, {self.y}) => ({centerCoordinates(self.x, self.y)[0]}, {centerCoordinates(self.x, self.y)[1]})")

        #TODO we don't need to loop up to self.x, we need to loop up to what self.x would be if it was at max length
        # because right now the vectors will shrink after hitting... something, except collision isn't working right,
        # but they never grow back up to full size after obstacles have passed

        # for each coordinate/pixel value in the vector
        # for x in frange(0, self.x, stepVal):
        # for x in frange(0, MAX_LENGTH, stepVal):
        for x in frange(0, MAX_LENGTH, 0.1):
            y = slope * x + 0 # y=mx+b, b value might need to be something different so leaving in here for now

            coords = centerCoordinates(round(x), round(y))[::-1]

            #FIXME stopgap measure to not kill my laptop, need to figure something out for this function
            if abs(x) > MAX_LENGTH or abs(y) > MAX_LENGTH:
                self.setPolar(self.angle, MAX_LENGTH)
                return

            # print(mask[centerCoordinates(round(x), round(y))])
            # print(f"({self.x}, {self.y}) => ({x}, {y}) => ({centerCoordinates(x, y)[0]}, {centerCoordinates(x, y)[1]})")
            # print(f"slope: {slope} | stepVal: {stepVal} | coords: {coords}")

            # if the pixel at that location is NOT empty space (ie it is an obstacle)
            if mask[coords].any() > 0:
                # then we've reached our new length, so update that
                self.setXY(x, y)
                return

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
        

        self.feelers[30].color = RED


        # start pointing straight
        self.heading_arrow = Vector()
        self.heading_arrow.setPolar(0, MAX_LENGTH)
        self.heading_arrow.color = GREEN
    
    def update(self):
        # reset our heading
        self.heading_arrow.setPolar(0, 0)

        for feeler in self.feelers:
            # make a vector, from the end of the current vector if it was at max length, to the end of the vector at its current length
            # in practice, because everything starts at (0, 0), just add 180 to the angle so it's pointing the opposite direction and set its length to the length of the error
            error_vec = Vector()
            error = MAX_LENGTH - feeler.length
            error_vec.setPolar((feeler.angle + 180) % 360, error)

            # print(type(self.heading_arrow))

            # add this vector to main heading arrow
            self.heading_arrow += error_vec
        
        #TODO I think there's something else we need to do?
    
    def draw(self, image):
        self.heading_arrow.draw(image)
        pass #TODO


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

    # for item in dir(image):
    #     print(item)

    # print(image.shape)

    if not ret:
        break # the end of the video

    frame += 1

    if frame < 150:
        continue # skip the first 100 frames because it's just the robot sitting there
    # elif frame == 152:
    #     cv2.imwrite("frame.png", image)
    
    # for now just use recorded threshold, so don't have to bother about cutting robot out and warpPerspective-ing
    mask = threshold(image)
    # image = cv2.bitwise_and(mask, image)
    # mask = image

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
    cv2.waitKey(0) #TODO do we want to make this match the 8 fps or something? or videoWrite and not bother with real-time output?

    # done = True
    if frame > 500:
        done = True

video.release()
cv2.destroyAllWindows()