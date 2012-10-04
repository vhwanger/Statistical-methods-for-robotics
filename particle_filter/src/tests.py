import unittest
import math
import pdb
import numpy as np
import matplotlib.pyplot as plt
from constants import MAX_DISTANCE_CM
from map_py import Map
from particle import SensorModel, ParticleFilter

class TestMapFunctions(unittest.TestCase):
    def setUp(self):
        self.wean_map = Map('../data/map/wean.dat')

    def testExpectedDistance(self):
        """
        Tests the expected distance function and plots what the rays its
        generating look like
        """
        x = 5860
        y = 5450
        theta = 3.08923277603
        (xs, ys, distances) = self.wean_map.expected_distance(x, y, theta)

        points = []
        for (i, distance) in zip(range(len(xs)), distances):
            pts = zip(xs[i], ys[i])

            # a hack to limit the length of the ray by filtering out points that
            # are farther than the computed distance
            ray = [p for p in pts if math.sqrt((p[0]-x/10)**2 + (p[1]-y/10)**2) <
                   distance/10]
            points.append(ray)

        plt.figure()
        plt.imshow(plt.imread('map.png'))

        plt.axis([0,800,0,800])
        for ray in points:
            plt.plot([p[0] for p in ray], [p[1] for p in ray], markersize=100)
        plt.show()

class TestSensorModel(unittest.TestCase):
    def setUp(self):
        self.fig = plt.figure(1)

    def testNewErrors(self):
        y = []
        x = np.arange(1, MAX_DISTANCE_CM + 1)
        for i in x:
            y.append(SensorModel.sample_observation(i, 2000))
        print SensorModel.sample_observation(8183,2000)
        print SensorModel.sample_observation(8182,2000)
        plt.plot(x, y)
        plt.show()


class TestParticleFilter(unittest.TestCase):
    def setUp(self):
        self.pf = ParticleFilter()

    def testInitialization(self):
        self.pf.run()

if __name__ == '__main__':
    unittest.main()

