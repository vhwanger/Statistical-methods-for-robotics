import matplotlib.pyplot as plt
from learning import parse_file
import pdb
from mpl_toolkits.mplot3d import Axes3D
from constants import (COORDS, X_COORD, Y_COORD, Z_COORD, LABEL, FEATURE_4, FEATURE_5)


if __name__ == '__main__':
    node_features = parse_file('oakland_part3_an_rf.node_features')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for feature in [FEATURE_4, FEATURE_5]:
        xs = [p[COORDS][X_COORD] for p in node_features if p[LABEL] == feature]
        ys = [p[COORDS][Y_COORD] for p in node_features if p[LABEL] == feature]
        zs = [p[COORDS][Z_COORD] for p in node_features if p[LABEL] == feature]
        ax.plot(xs, ys, zs, '.', markersize=1)

    plt.show()
