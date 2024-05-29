import tkinter
from tkinter import filedialog

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from math import pi

WAYPOINT_FILENAME = "data/waypoints.csv"

# class to handle gui callbacks and stuff
class WaypointHandler:
    #TODO we don't need to create a new axis do we? how do we replot data in a different color? and like modify just the data we want to and not the GPS data?
    def __init__(self, waypts, dir, ax):
        # waypoints dict {@see loadWaypointsFile()} for more info
        self.waypoints = waypts
        self.direction = dir # north or south
        self.index = 0 # index of the currently selected waypoint

        self.axis = ax # reference to the axis so we can draw on it

        # draw all the waypoints onto the screen now that we're set up
        self.redraw()
    
    def redraw(self):
        #TODO draw all the waypoints and then a circle around the currently selected waypoint
        # call l.set_ydata(ydata) or something followed by plt.draw() or something
        x = [pt[1] for pt in self.waypoints[direction]] # because pairs are (lat, lon) and lat is y, longitude is x
        y = [pt[0] for pt in self.waypoints[direction]]
        
        ax.plot(x, y)
    
    # callback to add a waypoint after the currently selected waypoint
    def add(self, event):
        #TODO figure out how to insert a waypoint or something here
        #FIXME does this need to be index+1?
        self.waypoints[self.direction].insert(self.index, w)

        # and then we updated so redraw
        self.redraw()

    # callback to remove the current waypoint
    def subtract(self, event):
        # remove the waypoint at the current index (ie the currently selected waypoint)
        self.waypoints[self.direction].pop(self.index)

        # redraw the waypoints so the user can tell the waypoint was deleted
        self.redraw()
    
    # callback to save waypoints file callback
    def save(self, event):
        # open the file in write mode
        with open(WAYPOINT_FILENAME, "w") as f:
            # write the headers
            f.write("label,lat,lon\n")

            # and then for every different list of waypoints we have in the file
            for name, listOfWaypts in self.waypoints:
                # and for every point in those lists
                for point in listOfWaypts:
                    # write that point to the file
                    #TODO use format strings or something because this is ugly
                    f.write(name + "," + str(point[0]) + "," + str(point[1]) + "\n")


# copied/pasted from https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/waypoints_file/autonav_ws/src/autonav_nav/src/astar.py
def loadWaypointsFile():
    waypoints = {}

    # read GPS waypoints data from file
    with open(WAYPOINT_FILENAME, "r") as f:
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

    # north or south, which changes which waypoints set we're editing
    direction = "none"

    # open and read the file
    with open(filepath, "r") as logfile:
        for idx, line in enumerate(logfile.readlines()):
            # data is logged in the CSV like timestamp, name, x, y, theta (heading), latitude, longitude

            lat = float(line.split(",")[5].strip())
            lon = float(line.split(",")[6].strip())
            
            # go ahead and throw those points in the list
            points[1].append(lon) # lon is x
            points[0].append(lat) # lat is y

            # if we're pretty far into the run, heading should be stable
            if idx == 200:
                # so we ought to be able to tell if we're heading north or south
                direction = getWaypointsDirection(float(line.split(",")[4]))

    return points, direction

# make the GUI (buttons and stuff, register event handlers, etc)
def createGui(fig, ax, waypts, direction):
    BUTTON_Y = 0.05
    BUTTON_WIDTH = 0.2
    BUTTON_HEIGHT = 0.075

    callback = WaypointHandler(waypts, direction, ax)

    # TODO figure out what this does
    fig.subplots_adjust(bottom=0.3)

    # create the add waypoint button
    plus_axis = fig.add_axes([0.4, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    plus_button = Button(plus_axis, "add waypoint")
    plus_button.on_clicked(callback.add)

    # create the subtract waypoint button
    minus_axis = fig.add_axes([0.7, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    minus_button = Button(minus_axis, "subtract waypoint")
    minus_button.on_clicked(callback.subtract)

    # create the 'save waypoints file' button to write the file back
    save_axis = fig.add_axes([0.1, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    save_button = Button(save_axis, "save waypoints")
    save_button.on_clicked(callback.save)

    return fig, ax, plus_button, minus_button


# draw the logged GPS points on the screen
def drawPlot(listOfPoints, ax):
    ax.plot(listOfPoints[0], listOfPoints[1])


def main():
    # plt.ion() #?????

    # needed for the file dialog box for choosing a run to be displayed (will be called in getFilepath())
    root = tkinter.Tk()
    root.withdraw()

    # load all the waypoints from the file
    waypts = loadWaypointsFile()

    # get the logged GPS points
    gpsData, direction = getGpsPoints(getFilepath())

    # boilerplate matplotlib, get our figure container and the axes so we can draw on them with the functions
    figure, axis = plt.subplots()

    # draw the GPS points on the frame
    drawPlot(gpsData, axis)

    # create the gui (and keep the references to objects so they don't go out of scope and get shot)
    f, a, pb, mb = createGui(figure, axis, waypts, direction)

    # show everything and have matplotlib run the gui event loop and call our callbacks and everything
    plt.show()


# standard-issue main check
if __name__ == "__main__":
    main()