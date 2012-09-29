import random
import matplotlib.pyplot as plt
import pdb
import numpy as np
from map import MAX_DISTANCE_CM, Map
from log import Log

LAMBDA_PARAM = .0005
PARTICLE_COUNT = 10

class ParticleFilter:
    def __init__(self):
        self.wean_map = Map('../data/map/wean.dat')
        log_file = Log('../data/log/robotdata1.log')
        self.log_entries = log_file.iterator()

        open_cells = self.wean_map.open_cells()
        self.particles = []
        for i in range(PARTICLE_COUNT):
            idx = random.randint(1, len(open_cells))
            self.particles.append(Particle(*open_cells[idx], map=self.wean_map))

    def run(self):
        pass


class Particle:
    def __init__(self, x, y, map, theta=None):
        self.x, self.y = (x, y)
        self.theta = random.randint(1, 360)
        self.map = map

    def __repr__(self):
        return "(x=%s, y=%s, theta=%s)" % (self.x, self.y, self.theta)

    def compute_weight(self, laser_data):
        pass


class SensorModel:
    """
    Creates each aspect of our hybrid sensor model
    """
    def __init__(self, expected_distance):
        self.x = np.arange(1,MAX_DISTANCE_CM)
        self.y = (.699* self.sensor_noise(expected_distance) + 
             .1* self.random_noise() +
             .2* self.short_noise(expected_distance) +
             .001 * self.max_noise())
        self.y /= sum(self.y)
        return

    def sample_observation(self, measured_distance):
        return self.y[measured_distance]

    def sensor_noise(self, expected_distance):
        """
        Gaussian noise centered around the expected distance
        """
        sigma = 200
        mu = expected_distance

        def gaussian(x):
            norm = 1/(sigma * np.sqrt(2 * np.pi))
            v = norm * np.exp(-.5*((x-mu)/sigma)**2)
            return v

        expvfunc = np.vectorize(gaussian)
        y = expvfunc(self.x)
        return y

    def random_noise(self):
        """
        Uniform noise across the entire range of readings
        """
        y = np.array([1 / MAX_DISTANCE_CM] * len(self.x))
        return y

    def short_noise(self, expected_distance):
        """
        Noise associated with random objects in from of the expected_distance
        """
        def vfunc(x):
            norm = 1/(1-np.exp(-LAMBDA_PARAM*expected_distance))
            if x < expected_distance:
                return LAMBDA_PARAM * np.exp(-1 * LAMBDA_PARAM * x) * norm
            else:
                return 0

        shortvfunc = np.vectorize(vfunc)
        y = shortvfunc(self.x)
        return y

    def max_noise(self):
        """
        Max noise at the very last reading possible
        """
        y = [0.] * len(self.x)
        y[-1] = 1.
        return np.array(y)

