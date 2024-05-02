import tkinter
from tkinter import filedialog
# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


# === file dialog box (copied/pasted from https://github.com/Team-OKC-Robotics/FRC-2023/blob/master/process_wpilib_logs.py)===
root = tkinter.Tk()
root.withdraw()

log_path = filedialog.askopenfilename()
# === /file dialog box ===

#FIXME
with open(log_path) as f:
    log = f.read().split("\n")

# === read waypoints.csv for existing waypoints ===
waypoints = {}

# read GPS waypoints data from file
with open("data/waypoints.csv", "r") as f:
    for line in f.readlines():
        label, lat, lon = line.split(",")

        try:
            waypoints[label].append((float(lat), float(lon)))
        except KeyError:
            waypoints[label] = []
            waypoints[label].append((float(lat), float(lon)))
# === /waypoints.csv ===


# display data
plt.figure(1)
plt.plot() #TODO
plt.show()

#TODO gui stuff
#TODO callbacks