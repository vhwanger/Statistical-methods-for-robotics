import unittest
import math
import pdb
import matplotlib.pyplot as plt
from map import Map

class TestMapFunctions(unittest.TestCase):
    def setUp(self):
        self.wean_map = Map('../data/map/wean.dat')

    def testExpectedDistance(self):
        """
        Tests the expected distance function and plots what the rays its
        generating look like
        """
        x = 2000
        y = 4150
        (xs, ys, distances) = self.wean_map.expected_distance(x, y, 30)

        points = []
        for (i, distance) in zip(range(len(xs)), distances):
            pts = zip(xs[i], ys[i])
            ray = [p for p in pts if math.sqrt((p[0]-x/10)**2 + (p[1]-y/10)**2) <
                   distance]
            points.append(ray)

        plt.imshow(plt.imread('map.png'))
        #for i in range(len(xs)):
        #    plt.plot(xs[i, 0:], ys[i,0:])

        for ray in points:
            plt.plot([p[0] for p in ray], [p[1] for p in ray])
        plt.axis([0,800,0,800])
        plt.show()

if __name__ == '__main__':
    unittest.main()

