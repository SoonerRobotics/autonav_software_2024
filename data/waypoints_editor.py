import tkinter
from tkinter import filedialog

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from math import pi

# copied/pasted from https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/waypoints_file/autonav_ws/src/autonav_nav/src/astar.py
def loadWaypointsFile():
    waypoints = {}

    # read GPS waypoints data from file
    with open("data/waypoints.csv", "r") as f:
        for line in f.readlines()[1:]: # skip the first line because that's the CSV headers
            label, lat, lon = line.split(",")

            try:
                # add the waypoint to the appropriate list in the dictionary
                waypoints[label].append((float(lat), float(lon)))
            except KeyError:
                # if it doesn't exist then create it
                waypoints[label] = []
                waypoints[label].append((float(lat), float(lon)))
    
    return waypoints


# detect which direction we're heading and return the right waypoint label
def getWaypointsDirection(theta): # expects theta to be in radians or something
    # code below is stolen (and very heavily modified) from autonav_nav/astar.py
    heading_degrees = abs(theta * 180 / pi)

    # gosh I love being able to condense Python into 1 line of code
    return "compSouth" if 120 < heading_degrees < 240 else "compNorth"


# gets a filepath as chosen by the user from a file dialong box
def getFilepath():
    # uses Tkinter, stolen code from https://github.com/Team-OKC-Robotics/FRC-2023/blob/master/process_wpilib_logs.py
    return filedialog.askopenfilename()


# get the gps points as a tuple of all the lons and then all the lats, so it can be matplotlib'd instantly
def getGpsPoints(filepath):
    # returns like ([x1, x2, x3], [y1, y2, y3])
    points = ([], [])

    # open and read the file
    with open(filepath, "r") as logfile:
        for line in logfile.readlines():
            
            points[0].append() #TODO throw a line.split() somewhere in here

    return points

def drawPlot(listOfWaypoints):
    # get our matplotlib set up
    fig, ax = plt.subplots()

    #TODO

def main():
    # needed for the file dialog box for choosing a run to be displayed
    root = tkinter.Tk()
    root.withdraw()

    waypts = loadWaypointsFile()

    gpsData = getGpsPoints(getFilepath())

    drawPlot()

if __name__ == "__main__":
    main()