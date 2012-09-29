import random
import pdb
import numpy as np
from constants import (MAX_DISTANCE_CM, ZMAX, ZHIT, ZNOISE, ZSHORT,
                       PARTICLE_COUNT, SHORT_NOISE_LAMBDA)
from map import Map
from log import Log, Laser, Odometry

DEBUG = True

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

        if DEBUG:
            print self.particles

    def normalize_particle_weights(self):
        total = 0
        for p in self.particles:
            total += p.weight
        
        for p in self.particles:
            p.weight /= total

        return

    def run(self):
        for log_entry in self.log_entries:
            if isinstance(log_entry, Laser):
                [p.compute_weight(log_entry) for p in self.particles]
                print str(self.particles) + "\n"
            elif isinstance(log_entry, Odometry):
                [p.move_by(log_entry) for p in self.particles]


class Particle:
    def __init__(self, x, y, map, theta=None):
        self.x, self.y = (x, y)
        self.theta = random.randint(1, 360)
        self.map = map
        self.weight = 1

    def __repr__(self):
        return "(x=%s, y=%s, theta=%s, weight=%s)" % (self.x, self.y,
                                                      self.theta, self.weight)

    def move_by(self, odometry_entry):
        return

    def compute_weight(self, laser_entry):
        # we don't need the first two returned objects from this.
        (_, __, expected_distances) = self.map.expected_distance(self.x, self.y,
                                                                 self.theta)
        
        self.weight = 1
        print "computing weight"
        for (exp_dist, act_dist) in zip(expected_distances,
                                        laser_entry.distances):
            sm = SensorModel(exp_dist)
            self.weight *= sm.sample_observation(act_dist)

        return

class SensorModel:
    """
    Creates each aspect of our hybrid sensor model
    """
    def __init__(self, expected_distance):
        self.x = np.arange(1,MAX_DISTANCE_CM, 10)
        self.y = (ZHIT * self.sensor_noise(expected_distance) + 
             ZNOISE * self.random_noise() +
             ZSHORT * self.short_noise(expected_distance) +
             ZMAX * self.max_noise())
        self.y /= sum(self.y)
        return

    def sample_observation(self, measured_distance):
        try:
            return self.y[measured_distance / 10]
        except IndexError:
            raise Exception("Trying to sample out of bounds: %s" %
                            measured_distance)

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
            norm = 1/(1-np.exp(-SHORT_NOISE_LAMBDA*expected_distance))
            if x < expected_distance:
                return (SHORT_NOISE_LAMBDA * np.exp(-1 * SHORT_NOISE_LAMBDA * x)
                        * norm)
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

