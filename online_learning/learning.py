from sklearn import svm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import numpy as np
import pdb
import svr
from constants import (COORDS, X_COORD, Y_COORD, Z_COORD, FEATURE_4, FEATURE_5)

def parse_file(filename):
    f = open(filename, 'r')
    lines = f.readlines()[3:]
    data = [line.split() for line in lines]
    node_features = []
    for node in data:
        label = int(node[4])
        features = map(float, node[5:])
        coord = map(float, node[0:3])
        node_features.append((label, features, coord))
    return node_features


if __name__ == '__main__':
    node_features = parse_file('oakland_part3_am_rf.node_features')
    node_features_2 = parse_file('oakland_part3_an_rf.node_features')
    
    svr = svr.SVR(node_features, (FEATURE_4, FEATURE_5))

    # pull in data from the other log to classify
    first_class = [p for p in node_features_2 if p[0] == FEATURE_4]
    second_class = [p for p in node_features_2 if p[0] == FEATURE_5]

    test_classes = first_class + second_class
    random.shuffle(test_classes)

    class_1200 = []
    class_1400 = []
    print "beginning classifying"
    for el in test_classes:
        if svr.predict(el[1]) < 0:
            class_1200.append(el)
        else:
            class_1400.append(el)

    xs = [p[COORDS][X_COORD] for p in class_1200]
    ys = [p[COORDS][Y_COORD] for p in class_1200]
    zs = [p[COORDS][Z_COORD] for p in class_1200]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, '.', markersize=1)

    xs = [p[COORDS][X_COORD] for p in class_1400]
    ys = [p[COORDS][Y_COORD] for p in class_1400]
    zs = [p[COORDS][Z_COORD] for p in class_1400]

    ax.plot(xs, ys, zs, '.', markersize=1)
    plt.show()
