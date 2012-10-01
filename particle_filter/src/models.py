"""
Module contains probability density functions for both sensor and odometry.
"""
import numpy as np
from constants import (MAX_DISTANCE_CM, ZMAX, ZHIT, ZNOISE, ZSHORT, HIT_SIGMA,
                       SHORT_NOISE_LAMBDA)

ALPHA_1 = 1
ALPHA_2 = 1
ALPHA_3 = 1
ALPHA_4 = 1
class MotionModel:
    """
    This model comes from the Probabilistic Robotics text in Chapter 5
    """
    @classmethod
    def sample_control(self, odometry):
        prev = odometry.prev_odometry
        rot1 = np.arctan2(odometry.y - prev.y,
                          odometry.x - prev.x) - odometry.theta
        trans = np.sqrt((odometry.x - prev.x)**2 + (odometry.y - prev.y)**2)
        rot2 = prev.theta - odometry.theta - rot1

        #rot1_hat = rot1 - 
    pass

class SensorModel:
    """
    Creates each aspect of our hybrid sensor model
    """
    @classmethod
    def sample_observation(self, meas_dist, expected_distance):
        value = (ZHIT * self.sensor_noise(meas_dist, expected_distance) +
                 ZNOISE * self.random_noise() +
                 ZSHORT * self.short_noise(meas_dist, expected_distance) +
                 ZMAX * self.max_noise(meas_dist))
        return value

    @classmethod
    def sensor_noise(self, meas_dist, expected_distance):
        """
        Gaussian noise centered around the expected distance
        """
        mu = expected_distance

        norm = 1/(HIT_SIGMA * np.sqrt(2 * np.pi))
        v = norm * np.exp(-.5*((meas_dist-mu)/HIT_SIGMA)**2)
        return v

    @classmethod
    def random_noise(self):
        """
        Uniform noise across the entire range of readings
        """
        y = 1. / MAX_DISTANCE_CM
        return y

    @classmethod
    def short_noise(self, meas_dist, expected_distance):
        """
        Noise associated with random objects in from of the expected_distance
        """
        norm = 1/(1-np.exp(-SHORT_NOISE_LAMBDA*expected_distance))
        if meas_dist < expected_distance:
            return (SHORT_NOISE_LAMBDA * np.exp(-1 * SHORT_NOISE_LAMBDA *
                                                meas_dist) * norm)
        else:
            return 0

    @classmethod
    def max_noise(self, meas_dist):
        """
        Max noise at the very last reading possible
        """
        return 1 if meas_dist == MAX_DISTANCE_CM else 0
