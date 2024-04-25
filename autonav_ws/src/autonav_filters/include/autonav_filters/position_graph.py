import matplotlib.pyplot as plt 
import numpy as np
import pandas as pd

if __name__ == "__main__":
    my_csv = pd.read_csv('/home/tony/autonav_logs/1713916573060/autonav_filters_pf.csv', delimiter=',', skiprows=1)
    manual_csv = pd.read_csv('/home/tony/autonav_logs/1714000908406/autonav_filters_pf.csv', delimiter=',', skiprows=1)
    manual_csv = pd.read_csv('/home/tony/autonav_logs/1714001655225/autonav_filters_pf.csv', delimiter=',', skiprows=1)
    my_csv = pd.read_csv('/home/tony/autonav_logs/1714002124612/autonav_filters_pf.csv', delimiter=',', skiprows=1)
    my_csv = pd.read_csv('/home/tony/autonav_logs/1714002772908/autonav_filters_pf.csv', delimiter=',', skiprows=1)
    my_csv = pd.read_csv('/home/tony/autonav_logs/1714003235234/autonav_filters_pf.csv', delimiter=',', skiprows=1)

    print(manual_csv)
    counter = 0
    counter_manual = 0
    for i in range(len(my_csv)):
        if my_csv.iloc[i, 1] == 0 or my_csv.iloc[i, 2] == 0:
            print(my_csv.iloc[i, 1])
            print(my_csv.iloc[i, 2])
            counter += 1
    
    for i in range(len(manual_csv)):
        if manual_csv.iloc[i, 1] == 0 or manual_csv.iloc[i,2] == 0:
            counter_manual += 1

    print(counter)
    my_csv = my_csv[counter:]
    manual_csv = manual_csv[counter_manual:]
    manual_pos_x = manual_csv.iloc[:, 1]
    manual_pos_y = manual_csv.iloc[:, 2]
    gps_x = my_csv.iloc[:, 1]
    gps_y = my_csv.iloc[:, 2]
    print(my_csv)
    print(gps_x)
    print(gps_y)


    simulation_waypoints = [(35.19510, -97.43823), (35.19505, -97.43823), (35.19492, -97.43824),(35.19485, -97.43824),(35.19471, -97.43824)]
    sim_xs = [-1 * x[0] for x in simulation_waypoints]
    sim_ys = [x[1] for x in simulation_waypoints]
    print(sim_xs)
    print(sim_ys)

    plt.figure(1)
    plt.plot(-1 * gps_x, gps_y, label='particle filter position')
    plt.scatter(sim_xs, sim_ys, color='r', label='gps waypoints')
    plt.legend()
    plt.figure(2)
    plt.plot(-1 * manual_pos_x, manual_pos_y)

    plt.legend()
    plt.show()