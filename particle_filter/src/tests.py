import unittest
import math
import pdb
import numpy as np
import matplotlib.pyplot as plt
from constants import MAX_DISTANCE_CM
from map_py import Map
from particle import SensorModel, ParticleFilter
from log import Odometry
from models import MotionModel

class TestMapFunctions(unittest.TestCase):
    def setUp(self):
        self.wean_map = Map('../data/map/wean.dat')

    def testExpectedDistance(self):
        """
        Tests the expected distance function and plots what the rays its
        generating look like
        """
        x = 4000
        y = 4140
        theta = -math.pi/2
        (xs, ys, distances) = self.wean_map.expected_distance(x, y, theta)
        print distances

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


class TestMotionModel(unittest.TestCase):
    def setUp(self):
        self.odometry = Odometry('O -91.694 -150.113998 -1.336922 4.000403')
        prev_odometry = Odometry('O -92.456001 -147.320007 -1.336922 3.850964')
        self.odometry.prev_odometry = prev_odometry

    def testMotionModel(self):
        plt.plot([self.odometry.prev_odometry.x],
                 [self.odometry.prev_odometry.y], '.g', markersize=20)
        plt.arrow(self.odometry.x, self.odometry.y,
                  .07*(math.cos(self.odometry.theta)),
                  (math.sin(self.odometry.theta)*.07), fc='g', ec='g',
                  head_width=.03,
                  head_length=.03)
        plt.plot([self.odometry.x],
                 [self.odometry.y], '.g', markersize=20)
        plt.arrow(self.odometry.prev_odometry.x, self.odometry.prev_odometry.y,
                  .07*(math.cos(self.odometry.prev_odometry.theta)),
                  (math.sin(self.odometry.prev_odometry.theta)*.07), fc='g', ec='g',
                  head_width=.03,
                  head_length=.03)
        next_xs = []
        next_ys = []
        for _ in range(500):
            rot1_hat, trans_hat, rot2_hat = MotionModel.sample_control(self.odometry)
            next_x = (self.odometry.prev_odometry.x + 
                      trans_hat * np.cos(self.odometry.prev_odometry.theta + rot1_hat))
            next_xs.append(next_x)
            next_y = (self.odometry.prev_odometry.y + 
                      trans_hat * np.sin(self.odometry.prev_odometry.theta + rot1_hat))
            next_ys.append(next_y)
            next_theta = self.odometry.prev_odometry.theta + rot1_hat + rot2_hat
            plt.arrow(next_x, next_y, .07*(math.cos(next_theta)),
                      (math.sin(next_theta)*.07), fc='c', ec='c',
                      head_width=.03,
                      head_length=.03)
        #plt.plot(next_xs, next_ys, '.c', markersize=5)
        plt.axis('equal')
        plt.show()

class TestParticleFilter(unittest.TestCase):
    def setUp(self):
        self.pf = ParticleFilter()

    def testInitialization(self):
        self.pf.run()

if __name__ == '__main__':
    unittest.main()

