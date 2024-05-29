import tkinter
from tkinter import filedialog

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from math import pi

class WaypointHandler:
    def __init__(self):
        self.waypoints = {}

        self.index = 0
    
    def add(self, event):
        print(event)
        print(dir(event))
        #TODO
        pass

    def subtract(self, event):
        #TODO
        pass

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
    # returns a tuple with ([x1, x2, x3, ...], [y1, y2, y3, ...])
    points = ([], [])

    # open and read the file
    with open(filepath, "r") as logfile:
        for line in logfile.readlines():
            # data is logged in the CSV like timestamp, name, x, y, theta (heading), latitude, longitude

            lat = float(line.split(",")[5].strip())
            lon = float(line.split(",")[6].strip())
            
            # go ahead and throw those points in the list
            points[1].append(lon) # lon is x
            points[0].append(lat) # lat is y

    return points

# make the GUI (buttons and stuff, register event handlers, etc)
def createGui(fig, ax):
    callback = WaypointHandler()

    # TODO figure out what this does
    fig.subplots_adjust(bottom=0.3)

    # create the add waypoint button
    plus_axis = fig.add_axes([0.4, 0.05, 0.2, 0.075])
    plus_button = Button(plus_axis, "add waypoint")
    plus_button.on_clicked(callback.add)

    # create the subtract waypoint button
    minus_axis = fig.add_axes([0.7, 0.05, 0.2, 0.075])
    minus_button = Button(minus_axis, "subtract waypoint")
    minus_button.on_clicked(callback.subtract)

    # go ahead and show what we've drawn so far
    # plt.show()

    # call l.set_ydata(ydata) or something followed by plt.draw() or something

    return fig, ax


# draw the logged GPS points on the screen
def drawPlot(listOfPoints, ax):
    ax.plot(listOfPoints[0], listOfPoints[1])


def main():
    # needed for the file dialog box for choosing a run to be displayed
    root = tkinter.Tk()
    root.withdraw()

    # load all the waypoints from the file
    waypts = loadWaypointsFile()

    # get the logged GPS points
    gpsData = getGpsPoints(getFilepath())

    #TODO
    figure, axis = plt.subplots()

    # draw the GPS points on the frame
    drawPlot(gpsData, axis)

    # create the gui
    createGui(figure, axis)

    plt.show()

    # main GUI loop
    #TODO


# standard-issue main check
if __name__ == "__main__":
    main()