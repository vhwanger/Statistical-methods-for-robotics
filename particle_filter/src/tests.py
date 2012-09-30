import unittest
import math
import pdb
import numpy as np
import matplotlib.pyplot as plt
from constants import MAX_DISTANCE_CM
from map import Map
from particle import SensorModel, ParticleFilter

class TestMapFunctions(unittest.TestCase):
    def setUp(self):
        self.wean_map = Map('../data/map/wean.dat')

    def testExpectedDistance(self):
        """
        Tests the expected distance function and plots what the rays its
        generating look like
        """
        x = 3000
        y = 4150
        (xs, ys, distances) = self.wean_map.expected_distance(x, y, 30)

        points = []
        for (i, distance) in zip(range(len(xs)), distances):
            pts = zip(xs[i], ys[i])

            # a hack to limit the length of the ray by filtering out points that
            # are farther than the computed distance
            ray = [p for p in pts if math.sqrt((p[0]-x/10)**2 + (p[1]-y/10)**2) <
                   distance]
            points.append(ray)

        plt.imshow(plt.imread('map.png'))

        for ray in points:
            plt.plot([p[0] for p in ray], [p[1] for p in ray])
        plt.axis([0,800,0,800])
        plt.show()

class TestSensorModel(unittest.TestCase):
    def setUp(self):
        self.fig = plt.figure(1)

    def testNewErrors(self):
        y = []
        x = np.arange(1, MAX_DISTANCE_CM + 1)
        for i in x:
            y.append(SensorModel.sample_observation(i, 4000))
        plt.plot(x, y)
        plt.show()


class TestParticleFilter(unittest.TestCase):
    def setUp(self):
        self.pf = ParticleFilter()

    def testInitialization(self):
        self.pf.run()

if __name__ == '__main__':
    unittest.main()

