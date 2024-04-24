import matplotlib.pyplot as plt 
import numpy as np
import pandas as pd

if __name__ == "__main__":
    my_csv = pd.read_csv('/home/tony/autonav_logs/1713916573060/autonav_filters_pf.csv', delimiter=',', skiprows=1)

    counter = 0
    for i in range(len(my_csv)):
        
        if my_csv.iloc[i, 1] == 0 or my_csv.iloc[i, 2] == 0:
            print(my_csv.iloc[i, 1])
            print(my_csv.iloc[i, 2])
            counter += 1

    print(counter)
    my_csv = my_csv[counter:]
    gps_x = my_csv.iloc[:, 1]
    gps_y = my_csv.iloc[:, 2]
    print(my_csv)
    print(gps_x)
    print(gps_y)

    plt.plot(gps_x, gps_y)
    plt.show()