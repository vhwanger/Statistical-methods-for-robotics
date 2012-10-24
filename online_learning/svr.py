import pdb
import random
import numpy as np
LAMBDA = .6

class SVR:
    def __init__(self, nodes, class_labels):
        """
        class_labels is a tuple describing the two labels you'd like to classify:
            class_labels = (1004, 1400)
        """
        self.nodes = self.filter_two_classes(nodes, class_labels)
        self.fit()
        return

    def filter_two_classes(self, nodes, class_labels):
        neg_class_label = class_labels[0]
        pos_class_label = class_labels[1]

        neg_class = [(-1, np.array(p[1])) for p in nodes if p[0]== neg_class_label]
        pos_class = [(1, np.array(p[1])) for p in nodes if p[0] == pos_class_label]

        all_classes = neg_class + pos_class
        random.shuffle(all_classes)
        return all_classes

    def fit(self):
        n_features = len(self.nodes[0][1])
        w = np.array([1] * n_features)
        ALPHA = 1/np.sqrt(len(self.nodes))

        counter = 1
        for label, feature in self.nodes:
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
