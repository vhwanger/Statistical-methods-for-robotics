import unittest
import math
import pdb
import matplotlib.pyplot as plt
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
        self.sm = SensorModel(4000)
        self.fig = plt.figure(1)

    def testErrors(self):
        y= self.sm.random_noise()
        plt.subplot(2, 2, 1)
        plt.title('Uniform error')
        plt.plot(self.sm.x,y) 

        y= self.sm.sensor_noise(4000)
        plt.subplot(2,2,2)
        plt.title('Gaussian sensor error')
        plt.plot(self.sm.x,y) 

        y = self.sm.short_noise(4000)
        plt.subplot(2,2,3)
        plt.title('Short distance error')
        plt.plot(self.sm.x,y) 

        y = self.sm.max_noise()
        plt.subplot(224)
        plt.title('Max reading error')
        plt.plot(self.sm.x,y) 

        plt.figure(2)
        y = self.sm.y
        plt.plot(self.sm.x, y)
        plt.show()

class TestParticleFilter(unittest.TestCase):
    def setUp(self):
        self.pf = ParticleFilter()

    def testInitialization(self):
        print self.pf.particles

if __name__ == '__main__':
    unittest.main()

