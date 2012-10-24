from sklearn import svm
import numpy as np
import pdb
import svr

LABEL = 0
FEATURES = 1

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
    
    svr = svr.SVR(node_features, (1004, 1400))
    first_class = [p for p in node_features if p[0] == 1004]
    second_class = [p for p in node_features if p[0] == 1400]

    print first_class[0]
    print second_class[0]
    for i in range(100):
        print svr.predict(first_class[i][1]), svr.predict(second_class[i][1])
