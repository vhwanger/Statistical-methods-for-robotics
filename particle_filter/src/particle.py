import matplotlib.pyplot as plt
import pdb
import numpy as np
from map import MAX_DISTANCE_CM

LAMBDA_PARAM = .0005

class Particle:
    def __init__(self, x, y, theta):
        pass


class SensorModel:
    def __init__(self):
        self.x = np.arange(1,MAX_DISTANCE_CM)

    def sensor_noise(self, expected_distance):
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
        y = np.array([1 / MAX_DISTANCE_CM] * len(self.x))
        return y

    def short_noise(self, expected_distance):
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
        y = [0.] * len(self.x)
        y[-1] = 1.
        return np.array(y)

    def sensor_model(self, measured_distance, expected_distance):
        y = (.33* self.sensor_noise(expected_distance) + 
             .33* self.random_noise() +
             .33* self.short_noise(expected_distance) +
             .001 * self.max_noise())
        y /= sum(y)
        return y

