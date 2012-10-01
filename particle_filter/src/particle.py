import random
import pdb
import matplotlib.pyplot as plt
import numpy as np
import math
from constants import PARTICLE_COUNT
from models import SensorModel, MotionModel
from map import Map
from log import Log, Laser, Odometry

DEBUG = True

class ParticleFilter:
    def __init__(self):
        """
        Initializes the map and whatever log file. Then initializes
        PARTICLE_COUNT number of particles. 
        """
        plt.ion()
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111)
        self.wean_map = Map('../data/map/wean.dat')
        log_file = Log('../data/log/robotdata1.log')
        self.log_entries = log_file.iterator()

        # initialize uniform random particles across all open cells
        open_cells = self.wean_map.open_cells()
        self.particles = []
        for i in range(PARTICLE_COUNT):
            idx = random.randint(0, len(open_cells))
            self.particles.append(Particle(*open_cells[idx], map=self.wean_map))

    def normalize_particle_weights(self):
        """
        Takes all particles in the filter and normalizes them.
        """
        total = 0
        for p in self.particles:
            total += p.weight
        
        for p in self.particles:
            p.weight /= total

        return

    def draw(self):
        resolution = self.wean_map.parameters['resolution']
        self.ax.clear()
        plt.axis([0,800,0,800])
        plt.imshow(plt.imread('map.png'))
        line, = plt.plot([p.x / resolution for p in self.particles],
                         [p.y / resolution for p in self.particles],
                         'g.',
                         markersize=10)
        plt.pause(.1)

    def run(self):
        """
        Iterates through all the log entries. If the log entry is a Laser object,
        it updates the weights of all the particles. If it's an Odometry object,
        it moves the particles.
        """
        for log_entry in self.log_entries:
            if isinstance(log_entry, Laser):
                [p.compute_weight(log_entry) for p in self.particles]
            elif isinstance(log_entry, Odometry):
                [p.move_by(log_entry) for p in self.particles]
            self.draw()


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

        if any([odometry.x != prev.x, odometry.y != prev.y, odometry.theta !=
                prev.theta]):
            #print "Previous pose: %s %s %s" % (prev.x, prev.y, prev.theta)
            #print "Noiseless pose: %s %s %s" % (odometry.x, odometry.y, odometry.theta)
            print "Previous pose: %s %s %s" % (self.x, self.y, self.theta)
            rot1_hat, trans_hat, rot2_hat = MotionModel.sample_control(odometry)
            self.x = (self.x + 
                      trans_hat * np.cos(prev.theta + rot1_hat))
            self.y = (self.y + 
                      trans_hat * np.sin(prev.theta + rot1_hat))
            self.theta = self.theta + rot1_hat + rot2_hat
            
            print "Noisy pose: %s %s %s" % (self.x, self.y, self.theta)
        else:
            print "no change in pose"
        return

    def compute_weight(self, laser_entry):
        """
        Takes in a laser log entry and calculates how well the particle matches
        the log entry.
        """
        # we don't need the first two returned objects from this.
        (_, __, expected_distances) = self.map.expected_distance(self.x, self.y,
                                                                 self.theta)
        
        self.weight = 1
        for (exp_dist, act_dist) in zip(expected_distances, laser_entry.distances):
            self.weight *= SensorModel.sample_observation(float(str(act_dist)), exp_dist)

        return

