import random
import time
import bisect
import pdb
import matplotlib.pyplot as plt
import numpy as np
import math
from constants import PARTICLE_COUNT, VARIANCE_THRESHOLD
from models import SensorModel, MotionModel
#try:
#    import pyximport; pyximport.install()
#    import map_c
#    CYTHON = True
#except:
import map_py
CYTHON = False
from log import Log, Laser, Odometry

DEBUG = True

class WeightedDistribution:
    def __init__(self, particles):
        accum = 0.0
        self.particles = [p for p in particles if p.weight > 0]
        self.r = random.uniform(0,1) * 1./len(self.particles) + 1e-15
        self.interval = 1. / len(self.particles)
        self.distribution = []
        for x in self.particles:
            accum += x.weight
            self.distribution.append(accum)

    def pick(self):
        try:
            index = self.particles[bisect.bisect_left(self.distribution,
                                                      random.uniform(0,1))]
            self.r += self.interval
            return index

        except IndexError:
            # Happens when all particles are improbable w=0
            print "all are zero"
            return None

def compute_weight_p(particle, laser_entry):
    """
    Takes in a laser log entry and calculates how well the particle matches
    the log entry.
    """
    try:
        if not particle.map.is_free((particle.x, particle.y)) or particle.weight == 0:
            return 0

        # we don't need the first two returned objects from this.
        (_, __, expected_distances) = particle.map.expected_distance(particle.x, particle.y,
                                                                 particle.theta)
        
        weight = 1
        for (exp_dist, act_dist) in zip(expected_distances, laser_entry.distances):
            weight *= (SensorModel.sample_observation(float(str(act_dist)),
                                                               exp_dist))**(1./len(expected_distances))
        return weight
    except KeyboardInterrupt:
        sys.exit(1)
    
    return
import multiprocessing
import itertools

try:
    cpus = multiprocessing.cpu_count()
except NotImplementedError:
    cpus = 2   # arbitrary default



def compute_weight_star(a_b):
    return compute_weight_p(*a_b)

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
        self.line, = plt.plot([], [], 'g.', markersize=5)

        # get data
        if CYTHON:
            self.wean_map = map_c.Map('../data/map/wean.dat')
        else:
            self.wean_map = map_py.Map('../data/map/wean.dat')
        log_file = Log('../data/log/robotdata1.log')
        self.log_entries = log_file.iterator()

        # initialize uniform random particles across all open cells
        self.open_cells = self.wean_map.open_cells()
        self.particles = []
        for i in range(PARTICLE_COUNT):
            self.particles.append(self.create_random())

    def create_random(self):
        idx = random.randint(0, len(self.open_cells)-1)
        return Particle(*self.open_cells[idx], map=self.wean_map)

    def normalize_particle_weights(self):
        """
        Takes all particles in the filter and normalizes them.
        """
        def assign_weight(particle, value):
            particle.weight /= float(value)

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
    
    def draw_arrows(self):
        resolution = self.wean_map.parameters['resolution']
        for p in self.particles:
            plt.arrow(p.x/resolution, p.y/resolution, (10*math.cos(p.theta)),
                      (10*math.sin(p.theta)), fc='c', ec='c', head_width=10,
                      head_length=10)

    def compute_variance(self):
        return np.var([p.weight for p in self.particles])

    def resample(self):
        print self.compute_variance()
        if self.compute_variance() > VARIANCE_THRESHOLD:
            print "resampling"
            dist = WeightedDistribution(self.particles)

            new_particles = []
            for _ in self.particles:
                p = dist.pick()
                if p is None:
                    new_particle = self.create_random()
                else:
                    new_particle = Particle(p.x, p.y, self.wean_map, p.theta)
                new_particles.append(new_particle)
            self.particles = new_particles
        return

    def reset_figure(self):
        self.ax.cla()

    def run(self, limit=None):
        """
        Iterates through all the log entries. If the log entry is a Laser object,
        it updates the weights of all the particles. If it's an Odometry object,
        it moves the particles.
        """
        pool = multiprocessing.Pool(processes=cpus)
        counter = 1
        for (i, log_entry) in enumerate(self.log_entries):
            print i
            if isinstance(log_entry, Laser):
                #self.ax.clear()
                #self.line, = plt.plot([], [], 'g.', markersize=5)
                #plt.imshow(plt.imread('map.png'))
                #plt.axis([0,800,0,800])
                #[p.compute_weight(log_entry) for p in self.particles]
                weights = pool.map(compute_weight_star, itertools.izip(self.particles,
                                                                    itertools.repeat(log_entry)))
                for (i, weight) in enumerate(weights):
                    self.particles[i].weight = weight

                #max_weight = max([p.weight for p in self.particles])
                #best_particle = [p for p in self.particles if p.weight == max_weight][0]
                #for p in self.particles:
                #    axis = p.print_actual_reading(log_entry, self.ax)
                #    print p.weight
                #    pdb.set_trace()
                #best_particle.print_actual_reading(log_entry, self.ax)
                #plt.draw()
                #pdb.set_trace()
                #for line in self.ax.lines:
                #    self.ax.lines.pop(0)
                #plt.draw()
                #self.draw()
                self.normalize_particle_weights()
                self.resample()
            elif isinstance(log_entry, Odometry):
                [p.move_by(log_entry) for p in self.particles]
                self.draw()
                if log_entry.prev_odometry is not None and log_entry.has_changed():
                    print "movement"
                    #if counter % 100 == 0:
                    #    self.draw_arrows()
                    #[p.draw_lasers() for p in self.particles]
                    #pdb.set_trace()
                    counter += 1
            if limit and limit == i:
                return
        return


class Particle:
    def __init__(self, x, y, map, theta=None):
        self.x, self.y = (x, y)
        if theta is None:
            self.theta = math.radians(random.randint(1, 360))
        else:
            self.theta = theta
        self.map = map
        self.weight = 1

    def __repr__(self):
        return "(x=%s, y=%s, theta=%s, weight=%s)" % (self.x, self.y,
                                                      self.theta, self.weight)

    def print_actual_reading(self, laser_reading, axis):
        (xs, ys, distances) = self.map.expected_distance(self.x, self.y,
                                                              self.theta)

        points = []
        for (i, distance) in zip(range(len(xs)), laser_reading.distances):
            pts = zip(xs[i], ys[i])

            # a hack to limit the length of the ray by filtering out points that
            # are farther than the computed distance
            ray = [p for p in pts if math.sqrt((p[0]-self.x/10)**2 + (p[1]-self.y/10)**2) <
                   distance/10]
            points.append(ray)

        for ray in points:
            axis.plot([p[0] for p in ray], [p[1] for p in ray], markersize=100)
        return axis



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
            #print "Previous pose: %s %s %s" % (self.x, self.y,
            #                                   math.degrees(self.theta))
            (self.prev_x, self.prev_y, self.prev_theta) = (self.x, self.y,
                                                           self.theta)
            rot1_hat, trans_hat, rot2_hat = MotionModel.sample_control(odometry)
            self.x = (self.x + 
                      trans_hat * np.cos(self.theta + rot1_hat))
            self.y = (self.y + 
                      trans_hat * np.sin(self.theta + rot1_hat))
            self.theta = self.theta + rot1_hat + rot2_hat
            if self.x < 0 or self.y < 0:
                print "WARNING - PARTICLE MOVED OFF MAP"
            
            #print "Noisy pose: %s %s %s" % (self.x, self.y,
            #                                math.degrees(self.theta))
        return

    def draw_lasers(self):
        (xs, ys, expected_distances) = self.map.expected_distance(self.x, self.y,
                                                                 self.theta)
        points = []
        for (i, distance) in zip(range(len(xs)), expected_distances):
            pts = zip(xs[i], ys[i])

            # a hack to limit the length of the ray by filtering out points that
            # are farther than the computed distance
            ray = [p for p in pts if math.sqrt((p[0]-self.x/10)**2 + (p[1]-self.y/10)**2) <
                   distance]
            points.append(ray)
        for ray in points:
            plt.plot([p[0] for p in ray], [p[1] for p in ray], markersize=100)
        plt.draw()

    def compute_weight(self, laser_entry):
        """
        Takes in a laser log entry and calculates how well the particle matches
        the log entry.
        """
        if not self.map.is_free((self.x, self.y)) or self.weight == 0:
            self.weight = 0
            return

        # we don't need the first two returned objects from this.
        (xs, ys, expected_distances) = self.map.expected_distance(self.x, self.y,
                                                                 self.theta)
        
        #self.draw_lasers()

        self.weight = 1
        for (exp_dist, act_dist) in zip(expected_distances, laser_entry.distances):
            sample = (SensorModel.sample_observation(float(str(act_dist)),
                                                     exp_dist))**(1./len(expected_distances))
            self.weight *= sample

        return





if __name__ == '__main__':
    pf = ParticleFilter()
    import cProfile, pstats
    cProfile.runctx('pf.run(20)', globals(), locals(), 'profile.prof')
    s = pstats.Stats("profile.prof")
    s.strip_dirs().sort_stats('time').print_stats()
