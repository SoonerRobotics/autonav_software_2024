import tkinter
from tkinter import filedialog

import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from matplotlib.widgets import Button
from matplotlib.backend_bases import MouseButton

from math import pi

WAYPOINT_FILENAME = "data/waypoints.csv"
BUTTON_Y = 0.05
BUTTON_WIDTH = 0.2
BUTTON_HEIGHT = 0.075


# class to handle gui callbacks and stuff
class WaypointHandler:
    #TODO we don't need to create a new axis do we? how do we replot data in a different color? 
    # and like modify just the data we want to and not the GPS data?
    def __init__(self):
        # load the waypoints from the file
        self.loadWaypointsFile()

        # get the logged GPS points
        # uses Tkinter, stolen code from https://github.com/Team-OKC-Robotics/FRC-2023/blob/master/process_wpilib_logs.py
        self.gpsData = self.getGpsPoints(filedialog.askopenfilename())

        self.index = 0 # index of the currently selected waypoint

        self.axis, self.figure = plt.subplots() # standard matplotlib code, get us some objects to draw graphics on
        self.firstDraw = True # if this is the first time we've drawn to the screen or not
        self.createGui() # go ahead and create the rest of the gui

        # draw all the waypoints onto the screen now that we're set up
        self.redraw()

    # copied/pasted from https://github.com/SoonerRobotics/autonav_software_2024/blob/feat/waypoints_file/autonav_ws/src/autonav_nav/src/astar.py
    # load the waypoints from the file
    def loadWaypointsFile(self):
        self.waypoints = {}

        # read GPS waypoints data from file
        with open(WAYPOINT_FILENAME, "r") as f:
            for line in f.readlines()[1:]: # skip the first line because that's the CSV headers
                label, lat, lon = line.split(",")

                try:
                    # add the waypoint to the appropriate list in the dictionary
                    self.waypoints[label].append((float(lat), float(lon)))
                except KeyError:
                    # if it doesn't exist then create it
                    self.waypoints[label] = []
                    self.waypoints[label].append((float(lat), float(lon)))

    # draw the logged GPS points on the screen
    #FIXME htis should be part of self.redraw() or something, we don't need a 1-line function
    def drawPlot(self):
        self.axis.plot(self.gpsData[0], self.gpsData[1], color="blue") #TODO make the color a constant or something

    # get the gps points as a tuple of all the lons and then all the lats, so it can be matplotlib'd instantly
    def getGpsPoints(self, filepath):
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
                    # detect which direction we're heading and return the right waypoint label
                    # code stolen and very very heavily modified and condensed from autonav_nav/astar.py
                    self.direction = "compSouth" if 120 < abs((float(line.split(",")[4])) * 180 / pi) < 240 else "compNorth"
        return points

    # make the GUI (buttons and stuff, register event handlers, etc)
    def createGui(self):
        # for reference code was based partially on https://matplotlib.org/stable/gallery/widgets/buttons.html
        # TODO figure out what this does
        self.figure.subplots_adjust(bottom=0.3)

        # create the add waypoint button
        self.plus_axis = self.fig.add_axes([0.4, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        self.plus_button = Button(self.plus_axis, "add waypoint")
        self.plus_button.on_clicked(self.add)

        # create the subtract waypoint button
        self.minus_axis = self.fig.add_axes([0.7, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        self.minus_button = Button(self.minus_axis, "subtract waypoint")
        self.minus_button.on_clicked(self.subtract)

        # create the 'save waypoints file' button to write the file back
        self.save_axis = self.fig.add_axes([0.1, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        self.save_button = Button(self.save_axis, "save waypoints")
        self.save_button.on_clicked(self.save)

        # binds an event handler so we can keep track of the mouse
        self.mosue_binding_id = plt.connect("button_press_event", callback.on_click)

        # draw the GPS points on the frame
        #FIXME this should be handled in redraw()???
        self.drawPlot(gpsData, axis)

        # show everything and have matplotlib run the gui event loop and call our callbacks and everything
        plt.show()

        print("GUI CREATED")

    # repaint the waypoints on the screen so the user can see what they are editing    
    def redraw(self):
        x = [pt[0] for pt in self.waypoints[self.direction]] # because pairs are (lat, lon) and lat is y, longitude is x
        y = [pt[1] for pt in self.waypoints[self.direction]] # exept not actually, it is currently correct, so I don't know what's going on

        # draw a black ring around the currently selected waypoint
        circle = plt.Circle((x[self.index], y[self.index]), 0.00001, color="black")
        self.axis.add_patch(circle) # https://stackoverflow.com/questions/9215658/plot-a-circle-with-pyplot
        
        # if it's the first time we're drawing,
        if self.firstDraw:
            # we need to plot everything
            self.axis.plot(x, y, linestyle="", marker=MarkerStyle("o"), color="#00FF44")

            print("FIRST DRAW!")
        else:
            # otherwise don't draw ghost waypoints that have been deleted, just set the y data or something idk
            axis.set_ydata(y) #FIXME this still leaves waypoints after they've been deleted
        
        plt.draw() # redraw to the screen (not plt.show() because that would be blocking and then we'd have problems)
    
    # callback to add a waypoint after the currently selected waypoint
    def add(self, event):
        #TODO figure out how to insert a waypoint or something here
        #FIXME does this need to be index+1?
        self.waypoints[self.direction].insert(self.index, w)

        print("WAYPOINT ADDED")

        # and then we updated so redraw
        self.redraw()

    # callback to remove the current waypoint
    def subtract(self, event):
        # remove the waypoint at the current index (ie the currently selected waypoint)
        self.waypoints[self.direction].pop(self.index)

        print("WAYPOINT SUBTRACTED")

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
                    f.write(f"{name},{point[1]},{point[0]}\n")
    
    # callback for mouse clicks
    def on_click(self, event):
        #TODO need to also make sure it's not pressing a button, maybe have an && self.inEditMode check in here somewhere?
        if event.button is MouseButton.LEFT and event.inaxes:
            print(f"CLICKED! ({event.xdata},{event.ydata}) || ({event.x},{event.y})")


def main():
    # needed for the file dialog box for choosing a run to be displayed
    root = tkinter.Tk()
    root.withdraw()

    # make an instance of the giant class to run everything and automatically run it
    application = WaypointHandler()

# standard-issue main check
if __name__ == "__main__":
    main()