"""
Visualizes the data straight from the logs
"""
import matplotlib.pyplot as plt
import pdb
from mpl_toolkits.mplot3d import Axes3D
from constants import (GROUND, FACADE)
from log import LogData

if __name__ == '__main__':
    log = LogData('oakland_part3_an_rf.node_features')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    log.plot_classes(ax, (GROUND, FACADE))

    plt.show()
