import random
import time
import bisect
import pdb
import matplotlib.pyplot as plt
import numpy as np
import math
from constants import PARTICLE_COUNT, VARIANCE_THRESHOLD
from models import SensorModel, MotionModel
from map import Map
from log import Log, Laser, Odometry

DEBUG = True

class WeightedDistribution:
    def __init__(self, particles):
        accum = 0.0
        self.particles = [p for p in particles if p.weight > 0]
        self.distribution = []
        for x in self.particles:
            accum += x.weight
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.particles[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None

class ParticleFilter:
    def __init__(self):
        """
        Initializes the map and whatever log file. Then initializes
        PARTICLE_COUNT number of particles. 
        """
        # initialize the drawing figure
        plt.ion()
        self.figure = plt.figure()
        plt.draw()
        plt.imshow(plt.imread('map.png'))
        plt.axis([0,800,0,800])
        self.ax = self.figure.add_subplot(111)
        self.line, = plt.plot([], [], 'g.', markersize=3)

        # get data
        self.wean_map = Map('../data/map/wean.dat')
        log_file = Log('../data/log/robotdata1.log')
        self.log_entries = log_file.iterator()

        # initialize uniform random particles across all open cells
        self.open_cells = self.wean_map.open_cells()
        self.particles = []
        for i in range(PARTICLE_COUNT):
            self.particles.append(self.create_random())

    def create_random(self):
        idx = random.randint(0, len(self.open_cells))
        return Particle(*self.open_cells[idx], map=self.wean_map)

    def normalize_particle_weights(self):
        """
        Takes all particles in the filter and normalizes them.
        """
        def assign_weight(particle, value):
            particle.weight /= value

        total = sum([p.weight for p in self.particles]) 
        [assign_weight(p, total) for p in self.particles]

        return

    def draw(self):
        resolution = self.wean_map.parameters['resolution']
        plt.axis([0,800,0,800])
        self.line.set_xdata([p.x / resolution for p in self.particles])
        self.line.set_ydata([p.y / resolution for p in self.particles])
        plt.draw()
        time.sleep(.001)

    def compute_variance(self):
        return np.var([p.weight for p in self.particles])

    def resample(self):
        print "resampling"
        print self.compute_variance()
        if self.compute_variance() > VARIANCE_THRESHOLD:
            dist = WeightedDistribution(self.particles)

            new_particles = []
            for _ in self.particles:
                p = dist.pick()
                if p is None:
                    new_particle = self.create_random()
                else:
                    new_particle = Particle(p.x, p.y, self.wean_map)
                new_particles.append(new_particle)
            self.particles = new_particles
        return

    def run(self):
        """
        Iterates through all the log entries. If the log entry is a Laser object,
        it updates the weights of all the particles. If it's an Odometry object,
        it moves the particles.
        """
        for (i, log_entry) in enumerate(self.log_entries):
            print i
            if isinstance(log_entry, Laser):
                [p.compute_weight(log_entry) for p in self.particles]
                self.normalize_particle_weights()
                self.resample()
            elif isinstance(log_entry, Odometry):
                [p.move_by(log_entry) for p in self.particles]
                if log_entry.prev_odometry is not None and log_entry.has_changed():
                    self.draw()
        return


class Particle:
    def __init__(self, x, y, map, theta=None):
        self.x, self.y = (x, y)
        self.theta = math.radians(random.randint(1, 360))
        self.map = map
        self.weight = 1

    def __repr__(self):
        return "(x=%s, y=%s, theta=%s, weight=%s)" % (self.x, self.y,
                                                      self.theta, self.weight)

    def move_by(self, odometry):
        """
        This takes in the change in odometry information calculated from the Log
        iterator function.
        """
        prev = odometry.prev_odometry
        if prev is None:
            return

        if odometry.has_changed():
            #print "Previous pose: %s %s %s" % (prev.x, prev.y, prev.theta)
            #print "Noiseless pose: %s %s %s" % (odometry.x, odometry.y, odometry.theta)
            #print "Previous pose: %s %s %s" % (self.x, self.y, self.theta)
            rot1_hat, trans_hat, rot2_hat = MotionModel.sample_control(odometry)
            self.x = (self.x + 
                      trans_hat * np.cos(self.theta + rot1_hat))
            self.y = (self.y + 
                      trans_hat * np.sin(self.theta + rot1_hat))
            self.theta = self.theta + rot1_hat + rot2_hat
            
            #print "Noisy pose: %s %s %s" % (self.x, self.y, self.theta)
        return

    def compute_weight(self, laser_entry):
        """
        Takes in a laser log entry and calculates how well the particle matches
        the log entry.
        """
        # we don't need the first two returned objects from this.
        if not self.map.is_free((self.x, self.y)):
            self.weight = 0
            return

        (_, __, expected_distances) = self.map.expected_distance(self.x, self.y,
                                                                 self.theta)
        
        self.weight = 1
        #for (exp_dist, act_dist) in zip(expected_distances, laser_entry.distances):
        #    self.weight *= SensorModel.sample_observation(float(str(act_dist)), exp_dist)
        # TAKE OUT
        self.weight = .9

        return

