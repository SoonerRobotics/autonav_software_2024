import tkinter
from tkinter import filedialog
# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# https://matplotlib.org/stable/gallery/event_handling/poly_editor.html
# https://matplotlib.org/stable/gallery/widgets/buttons.html
# is cool too https://matplotlib.org/stable/gallery/widgets/cursor.html
# this is almost what we need? https://matplotlib.org/stable/gallery/event_handling/path_editor.html
# https://github.com/SoonerRobotics/autonav_software_2024/blob/f1738c778a39729ceb984aa869a9ae290d2b720f/autonav_ws/src/autonav_filters/include/autonav_filters/position_graph.py
# https://github.com/SoonerRobotics/autonav_software_2024/blob/f1738c778a39729ceb984aa869a9ae290d2b720f/autonav_ws/src/autonav_filters/src/particlefilter.py


class GuiHandler():
    def __init__(self):
        # I think everything needs to be class variables (think static, eg declared outside this function in class definition) instead of instance variables? idk
        pass

    def delete_callback(self, event):
        pass


# === file dialog box (copied/pasted from https://github.com/Team-OKC-Robotics/FRC-2023/blob/master/process_wpilib_logs.py)===
root = tkinter.Tk()
root.withdraw()

log_path = filedialog.askopenfilename()
# === /file dialog box ===

# alright so we're gonna store the data like this:
# [
#   [time1, time2, time3, ...],
#   [x1, x2, x3, ...],
#   [y1, y2, y3],
#   [theta1, theta2, theta3],
#   ...
# ]
# to hopefully make it easier to plot
data = [
    [],
    [],
    [],
    [],
    [],
    [],
    []
]

#FIXME
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




# === read waypoints.csv for existing waypoints ===
#TODO throw these in a dropdown list or something? unless we want to parse the master log.csv and see if we log the waypoints we used. idk.
waypoints = {}

# read GPS waypoints data from file
with open("data/waypoints.csv", "r") as f:
    for line in f.readlines()[1:]: # skip the first line because that's the CSV headers
        label, lat, lon = line.split(",")

        try:
            waypoints[label].append((float(lat), float(lon)))
        except KeyError:
            waypoints[label] = []
            waypoints[label].append((float(lat), float(lon)))
# === /waypoints.csv ===

LABEL = "compNorth" #FIXME

waypoints_x = []
waypoints_y = []

for lat, lon in waypoints[LABEL]:
    waypoints_y.append(lat)
    waypoints_x.append(lon)

# display logged robot data
plt.figure()

# data[5] is lat, data[6] is lon, but matplotlib expects (x, y), so it's x=lon and y=lat (aka 6 then 5) instead of (data[5], data[6]) being passed into here
plt.scatter(data[6], data[5])
plt.scatter(waypoints_x, waypoints_y) # scatter the waypoints on there (temporary until they are editable)

#FIXME ???
# handler = GuiHandler()
# fig, ax = plt.subplots(2) #FIXME ???
# 
# ax1 = fig.add_axes((1, 1, 1, 1)) #FIXME
# deleteButton = Button(ax1, 'delete')
# deleteButton.on_clicked(handler.delete_callback)
#TODO gui stuff
#TODO callbacks

plt.show()