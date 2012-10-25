"""
Module for support vector machine
"""
import pdb
import numpy as np
import matplotlib.pyplot as plt
import pdb
from mpl_toolkits.mplot3d import Axes3D
from log import LogData
from constants import (COORDS, X_COORD, Y_COORD, Z_COORD, GROUND, FACADE)
LAMBDA = .9

class SVM:
    def __init__(self, logdata, class_labels):
        """
        class_labels is a tuple describing the two labels you'd like to classify:
            class_labels = (1004, 1400)
        """
        self.nodes = logdata.filter_two_classes(class_labels)
        self.normalize_features()
        self.fit()
        return

    def normalize_features(self):
        mean_v = sum([p[1] for p in self.nodes])/len(self.nodes)
        std_dev = np.std([p[1] for p in self.nodes])
        self.nodes = [(p[0], (p[1]- mean_v)/std_dev, p[2]) for p in self.nodes]
        return

    def fit(self):
        n_features = len(self.nodes[0][1])
        w = np.array([1] * n_features)
        ALPHA = 1/np.sqrt(len(self.nodes))

        counter = 1
        for label, feature, _ in self.nodes:
            #ALPHA = 1./np.sqrt(counter)
            margin_value = 1 - label * np.dot(w, feature)

            # if we don't meet the hard margin constraint
            if margin_value > 0:
                w_t_1 = w - ALPHA * LAMBDA * w + ALPHA * label * feature
            else:
                w_t_1 = w - ALPHA * LAMBDA * w
                pass
            w = w_t_1
            counter += 1
            pass
        self.w = w
        return

    def predict(self, feature):
        return np.dot(self.w, feature)

    def predict_and_plot(self, test_data, ax):
        neg_class = []
        pos_class = []
        for el in test_data:
            if self.predict(el[1]) < 0:
                neg_class.append(el)
            else:
                pos_class.append(el)

        xs = [p[COORDS][X_COORD] for p in neg_class]
        ys = [p[COORDS][Y_COORD] for p in neg_class]
        zs = [p[COORDS][Z_COORD] for p in neg_class]

        ax.plot(xs, ys, zs, '.', markersize=3)

        xs = [p[COORDS][X_COORD] for p in pos_class]
        ys = [p[COORDS][Y_COORD] for p in pos_class]
        zs = [p[COORDS][Z_COORD] for p in pos_class]

        ax.plot(xs, ys, zs, '.', markersize=3)

if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # we train on log1, then classify log2
    log1 = LogData('oakland_part3_am_rf.node_features')
    log2 = LogData('oakland_part3_an_rf.node_features')
    
    svm = SVM(log1, (GROUND, FACADE))
    test_classes = log2.filter_data((GROUND, FACADE))
    svm.predict_and_plot(test_classes, ax)

    plt.show()
