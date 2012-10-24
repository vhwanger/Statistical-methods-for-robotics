import matplotlib.pyplot as plt
from learning import parse_file
import pdb
from mpl_toolkits.mplot3d import Axes3D

LABEL = 0
FEATURES = 1
COORDS = 2
X_COORD = 0
Y_COORD = 1
Z_COORD = 2
FEATURES = [1004, 1100, 1103, 1200, 1400]
FEATURE_1 = 1004
FEATURE_2 = 1100
FEATURE_3 = 1103
FEATURE_4 = 1200
FEATURE_5 = 1400


if __name__ == '__main__':
    node_features = parse_file('oakland_part3_am_rf.node_features')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for feature in FEATURES:
        xs = [p[COORDS][X_COORD] for p in node_features if p[LABEL] == feature]
        ys = [p[COORDS][Y_COORD] for p in node_features if p[LABEL] == feature]
        zs = [p[COORDS][Z_COORD] for p in node_features if p[LABEL] == feature]
        ax.plot(xs, ys, zs, '.', markersize=1)

    plt.show()
