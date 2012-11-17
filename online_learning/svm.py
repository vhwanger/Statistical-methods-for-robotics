"""
Module for support vector machine
"""
import pdb
import numpy as np
import matplotlib.pyplot as plt
import pdb
import itertools
from mpl_toolkits.mplot3d import Axes3D
from log import LogData
from constants import (COORDS, X_COORD, Y_COORD, Z_COORD, VEG, WIRE, POLE,
                       GROUND, FACADE)
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
        """
        self.nodes = [ (label, [features]), ... ]
        """
        features = np.array([p[1] for p in self.nodes])
        features = features - np.mean(features, axis=0)
        features = features / np.std(features, axis=0)
        for (idx, p) in enumerate(self.nodes):
            self.nodes[idx] = (self.nodes[idx][0], features[idx],
                               self.nodes[idx][2])
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

    def predict_and_plot(self, test_data, ax, label_order):
        """
        test_data is a data structure that looks like:
            [(label, features), ...]
        """
        neg_class = []
        pos_class = []
        wrong_labels = 0
        labels = {-1: label_order[0],
                  1: label_order[1]}
        for el in test_data:
            prediction = -1 if self.predict(el[1]) < 0 else 1
            if labels[prediction] != el[0]:
                wrong_labels += 1

            if prediction < 0:
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

        print "Comparing %s" % str(label_order)
        print "Accuracy: %.1f%%" % ((1 - wrong_labels / (1. * len(test_data))) *
                                100)

if __name__ == '__main__':

    # we train on log1, then classify log2
    log1 = LogData('oakland_part3_am_rf.node_features')
    log2 = LogData('oakland_part3_an_rf.node_features')
    
    labels = [GROUND, FACADE, POLE, WIRE, VEG]
    
    for combo in itertools.combinations(labels, 2):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        svm = SVM(log1, combo)
        label_order = combo
        test_classes = log2.filter_data(label_order)
        svm.predict_and_plot(test_classes, ax, label_order)

    plt.show()
