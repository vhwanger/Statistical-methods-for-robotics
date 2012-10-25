"""
Module for support vector machine
"""
import pdb
import numpy as np
from constants import (COORDS, X_COORD, Y_COORD, Z_COORD)
LAMBDA = .6

class SVM:
    def __init__(self, logdata, class_labels):
        """
        class_labels is a tuple describing the two labels you'd like to classify:
            class_labels = (1004, 1400)
        """
        self.nodes = logdata.filter_two_classes(class_labels)
        self.fit()
        return

    def fit(self):
        n_features = len(self.nodes[0][1])
        w = np.array([1] * n_features)
        ALPHA = 1/np.sqrt(len(self.nodes))

        counter = 1
        for label, feature, _ in self.nodes:
            ALPHA = 1./np.sqrt(counter)
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

        ax.plot(xs, ys, zs, 'g.', markersize=3)

        xs = [p[COORDS][X_COORD] for p in pos_class]
        ys = [p[COORDS][Y_COORD] for p in pos_class]
        zs = [p[COORDS][Z_COORD] for p in pos_class]

        ax.plot(xs, ys, zs, '.', markersize=3)


