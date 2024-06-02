import tkinter
from tkinter import filedialog

import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from matplotlib.widgets import Button
from matplotlib.backend_bases import MouseButton

from math import pi

# DEBUG = False
# WAYPOINT_FILENAME = "data/waypoints.csv"

BUTTON_Y = 0.05
BUTTON_WIDTH = 0.15
BUTTON_HEIGHT = 0.075
RECT_WIDTH = 0.00002

GPS_DATA_COLOR = "blue"
SELECT_COLOR = "red"
WAYPT_COLOR = "orange"

def dist(p1, p2):
    return ((p1[0] - p2[0])**2  +  (p1[1] - p2[1])**2) ** 0.5 # raising to power of 1/2 is the same as sqrt

# class to handle gui callbacks and stuff
class WaypointHandler:
    def __init__(self):
        self.folderName = getFolderName() # base folder name of all the logged auton data

        self.figure, self.axis = plt.subplots() # standard matplotlib code, get us some objects to draw graphics on
        self.createGui() # go ahead and create the rest of the gui

        # draw all the waypoints onto the screen now that we're set up
        self.redraw()

        # everything should be all set up now, so we can
        # show everything and have matplotlib run the gui event loop and call our callbacks and everything
        plt.show()

    def mainLoop(self):
        pass #TODO


    # uses Tkinter, stolen code from https://github.com/Team-OKC-Robotics/FRC-2023/blob/master/process_wpilib_logs.py
    def getFolderName(self):
        return filedialog.askopenfilename()

    # get the gps points as a tuple of all the lons and then all the lats, so it can be matplotlib'd instantly
    def getGpsPoints(self, filepath):
        # returns a tuple with ([x1, x2, x3, ...], [y1, y2, y3, ...])
        points = ([], [])

        # open and read the file
        with open(filepath, "r") as logfile:
            for idx, line in enumerate(logfile.readlines()):
                # data is logged in the CSV like timestamp, name, x, y, theta (heading), latitude, longitude

                lat = float(line.split(",")[5].strip())
                lon = float(line.split(",")[6].strip())
                
                # go ahead and throw those points in the list
                points[1].append(lon) # lon is x
                points[0].append(lat) # lat is y

        return points

    # make the GUI (buttons and stuff, register event handlers, etc)
    def createGui(self):
        # for reference code was based partially on https://matplotlib.org/stable/gallery/widgets/buttons.html
        # TODO figure out what this does
        self.figure.subplots_adjust(bottom=0.3)

        # create the add waypoint button
        self.plus_axis = self.figure.add_axes([0.05 * 3 + BUTTON_WIDTH * 2, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        self.plus_button = Button(self.plus_axis, "add waypoint")
        self.plus_button.on_clicked(self.add)

        # create the subtract waypoint button
        self.minus_axis = self.figure.add_axes([0.05 * 4 + BUTTON_WIDTH * 3, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        self.minus_button = Button(self.minus_axis, "subtract waypoint")
        self.minus_button.on_clicked(self.subtract)

        # create the move waypoint button
        self.edit_axis = self.figure.add_axes([0.05 * 2 + BUTTON_WIDTH, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        self.edit_button = Button(self.edit_axis, "move waypoint")
        self.edit_button.on_clicked(self.edit)

        # create the 'save waypoints file' button to write the file back
        self.save_axis = self.figure.add_axes([0.05, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        self.save_button = Button(self.save_axis, "save waypoints")
        self.save_button.on_clicked(self.save)

        # binds an event handler so we can keep track of the mouse
        self.mosue_binding_id = plt.connect("button_press_event", self.on_click)

    # repaint the waypoints on the screen so the user can see what they are editing    
    def redraw(self):
        # clear existing data
        self.axis.clear()

        # draw the logged GPS points on the screen
        self.axis.plot(self.gpsData[0], self.gpsData[1], color=GPS_DATA_COLOR)

        x = [pt[0] for pt in self.waypoints[self.direction]] # because pairs are (lat, lon) and lat is y, longitude is x
        y = [pt[1] for pt in self.waypoints[self.direction]] # exept not actually, it is currently correct, so I don't know what's going on

        # draw a black ring around the currently selected waypoint
        circle = plt.Rectangle((x[self.index] - RECT_WIDTH/2, y[self.index] - RECT_WIDTH/2), RECT_WIDTH, RECT_WIDTH, color=SELECT_COLOR)
        self.axis.add_patch(circle) # https://stackoverflow.com/questions/9215658/plot-a-circle-with-pyplot
        
        # plot everything
        self.axis.plot(x, y, linestyle="", marker=MarkerStyle("o"), color=WAYPT_COLOR, markersize=10)
        
        plt.draw() # redraw to the screen (not plt.show() because that would be blocking and then we'd have problems)
    
    # callback to add a waypoint after the currently selected waypoint
    def add(self, event):
        # insert a waypoint after the current waypoint, at the same location
        self.waypoints[self.direction].insert(self.index + 1, self.waypoints[self.direction][self.index])

        # and then we updated so redraw
        self.redraw()

    
    # callback for mouse clicks (needed for moving and selecting waypoints)
    def on_click(self, event):
        if DEBUG:
            print(event.x, event.y, event.xdata, event.ydata)

        # if it's a right click and in bounds of the graph, and also not one of the buttons (all latitudes are around -83, so < 0, but the buttons are at like .5, which is why that check is there)
        # right click so we don't reset the screen when the user is trying to zoom in with the magnifying glass
        if event.button is MouseButton.RIGHT and event.inaxes and event.ydata < 0:
            # if we're editing (moving) a waypoint
            if self.inEditMode:
                # assign the coordinates of the mouse click to the currently selected waypoint
                self.waypoints[self.direction][self.index] = (event.xdata, event.ydata)

                #TODO reorder waypoints

                # take us out of edit mode, as the waypoint has now been moved
                self.inEditMode = False

                if DEBUG:
                    print(f"WAYPOINT EDITED ({event.xdata},{event.ydata})")
            # otherwise, the user is trying to select a waypoint, so do that
            else:
                bestIndex = 0 # default to the first waypoint

                # find whichever waypoint is closest
                for idx, waypoint in enumerate(self.waypoints[self.direction]):
                    # if the distance to the currently iterated waypoint is less than the closest distance to the cursor click event we've found so far, update it
                    if dist(waypoint, (event.xdata, event.ydata)) < dist(self.waypoints[self.direction][bestIndex], (event.xdata, event.ydata)):
                        bestIndex = idx
                
                # and update it so the user has selected whichever waypoint is closest to the cursor
                self.index = bestIndex

            # and we updated so redraw
            self.redraw()


def main():
    # needed for the file dialog box for choosing a run to be displayed
    root = tkinter.Tk()
    root.withdraw()

    # make an instance of the giant class to run everything and automatically run it
    application = WaypointHandler()

# standard-issue main check
if __name__ == "__main__":
    main()