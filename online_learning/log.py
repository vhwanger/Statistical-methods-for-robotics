"""
Module for processing the given log data. Bundles in parsing and some generic
functions for manipulating the data.
"""
import numpy as np
import pdb
import random
from constants import (COORDS, X_COORD, Y_COORD, Z_COORD)

class LogData:
    def __init__(self, filename):
        self.data = self.parse_file(filename)
        return

    def parse_file(self, filename):
        """
        Returns an organized data structure of the form:
            [ ( label, [ features ...], [x y z]), ... ]
        """
        f = open(filename, 'r')

        # skip the header
        lines = f.readlines()[3:]

        data = [line.split() for line in lines]
        nodes = []
        for node in data:
            label = int(node[4])
            features = map(float, node[5:])
            coord = map(float, node[0:3])
            nodes.append((label, features, coord))
        return nodes

    def filter_two_classes(self, class_labels):
        """
        Input is a tuple or list containing the two labels you want to filter
        out:
            class_labels = (1004, 1400)

        Output is a filtered list containing data only from the two specified
        classes. The labels are also changed so that the first specified class
        is labeled -1, and the second is 1. For example, having the above
        parameter would return something like:

            all_classes = [(-1, np.array[features], np.array[coords]), ...]

        """
        neg_class_label = class_labels[0]
        pos_class_label = class_labels[1]

        neg_class = [(-1, np.array(p[1]), np.array(p[2])) for p in self.data if 
                     p[0] == neg_class_label]
        pos_class = [(1, np.array(p[1]), np.array(p[2])) for p in self.data if 
                     p[0] == pos_class_label]

        all_classes = neg_class + pos_class
        random.shuffle(all_classes)
        return all_classes

    def filter_data(self, class_labels):
        """
        Input is a tuple or list containing the all the classes you'd like to be
        placed into the output list. For example, if you'd like all the data
        points for GROUND, FACADE, and WIRE, you'd do:

            log.filter_data((GROUND, FACADE, WIRE))

        And you'd get back something like:

            [(1400, [features], [coords]), (1004, [features], [coords]), ...]

        """
        nodes = []
        for label in class_labels:
            nodes += [p for p in self.data if p[0] == label]
        random.shuffle(nodes)
        return nodes

    def plot_classes(self, ax, class_labels):
        """
        Given a plt axis and a set of labels of the form:

            log.plt(ax, (GROUND, FACADE, WIRE))

        it will plot each class onto the given axis. They will be different
        colors as well.
        """
        for label in class_labels:
            data = self.filter_data((label,))

            xs = [p[COORDS][X_COORD] for p in data]
            ys = [p[COORDS][Y_COORD] for p in data]
            zs = [p[COORDS][Z_COORD] for p in data]

            ax.plot(xs, ys, zs, '.', markersize=3)
