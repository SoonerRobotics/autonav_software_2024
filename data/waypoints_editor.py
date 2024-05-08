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

def drawPlot(listOfWaypoints):
    pass

def main():
    waypts = loadWaypointsFile()

    drawPlot()

    registerDropdownCallback()

# detect which direction we're heading and return the right waypoint label
def getWaypointsDirection(run_waypoints):
    # code below is stolen from autonav_nav/astar.py
    heading_degrees = abs(run_waypoints[10] * 180 / pi)

    # if it's mostly on the south side of the compass
    if heading_degrees > 120 and heading_degrees < 240:
        north = False # then it's south
    else:
        north = True # otherwise north

    return "compNorth" if north else "compSouth"

if __name__ == "__main__":
    main()



# === file dialog box (copied/pasted from https://github.com/Team-OKC-Robotics/FRC-2023/blob/master/process_wpilib_logs.py)===
root = tkinter.Tk()
root.withdraw()

log_path = filedialog.askopenfilename()
# === /file dialog box ===

# open the log csv file
with open(log_path) as f:
    # # for each line in the file (no headers so don't have to worry about those)
    # for line in f.readlines():
    #     # the 2nd column is just "ENTRY_POSITION" so we don't need to worry about it
    #     timestamp, _, x, y, theta, lat, lon = line.split(",")

    # for each line in the file,
    for line in f.readlines():
        # for each column index
        for i in range(7):
            try:
                # append that column's data (the line.split(",")[i]) to the master data list
                data[i].append(float(line.split(",")[i]))
            except Exception as e:
                # except we're gonna run into the pesky 2nd column which is just "ENTRY_POSITION" and trying to float() that isn't gonna be pretty so skip it
                pass






# display logged robot data
# plt.figure()

# img = plt.imread("data/igvcMap.png")
fig, ax = plt.subplots()
# ax.imshow(img, origin="upper", extent=(-83.218835, -83.21944, 42.667870, 42.66835))
# 
# data[5] is lat, data[6] is lon, but matplotlib expects (x, y), so it's x=lon and y=lat (aka 6 then 5) instead of (data[5], data[6]) being passed into here
ax.scatter(data[6], data[5])
ax.scatter(waypoints_x, waypoints_y) # scatter the waypoints on there (temporary until they are editable)

LABEL = "givenWaypts"
waypoints_x = []
waypoints_y = []
for lat, lon in waypoints[LABEL]:
    waypoints_y.append(lat)
    waypoints_x.append(lon)
ax.scatter(waypoints_x, waypoints_y, color="#00FF88") # scatter the waypoints on there (temporary until they are editable)

plt.show()