"""
Classes for organizing different log files.
"""
import pdb
from decimal import Decimal
from constants import LIDAR_ANGLE_INTERVAL
D = Decimal

class Odometry:
    """
    Odometry data type. Contains x, y, theta, and timestamp data, all in
    reference to the standard odometry frame.
    """
    def __init__(self, line):
        self.parse(line)

    def __repr__(self):
        return "Odometry: %s %s %s %s" % (self.x, self.y, self.theta, self.ts)

    def parse(self, line):
        """
        Each line looks like:

        O -94.234001 -139.953995 -1.342158 0.025863
        """
        self.x, self.y, self.theta, self.ts = map(float, line.split()[1:])


class Laser:
    """
    Laser data type. Contains x, y, theta (odometry frame), and x, y, and theta
    in the laser frame.  Also contains a timestamp.
    """
    def __init__(self, line):
        self.parse(line)

    def __repr__(self):
        return "Laser: %s %s %s %s" % (self.x, self.y, self.theta, self.ts)

    def parse(self, line):
        """
        Each line looks like:
        L -94.234001 -139.953995 -1.342158 -88.567719 -164.303391 -1.342158 66
        66 66 ....
        """
        data = map(float, line.split()[1:])
        self.distances = data[6:-1]

        # let's remove the angle readings we don't care about
        self.distances = self.distances[::LIDAR_ANGLE_INTERVAL]

        self.x, self.y, self.theta = data[0:3]
        self.x_l, self.y_l, self.theta_l = data[3:6]
        self.ts = data[-1]


class Log:
    """
    Populated with odometry and laser data by initializing with the filename of
    a log file.
    """
    def __init__(self, filename):
        self.data = []

        f = open(filename, 'r')
        lines = f.readlines()
        self.parse(lines)
        self.prev_odometry = None

    def parse(self, lines):
        for line in lines:
            if line[0] == "O":
                self.data.append(Odometry(line))
            elif line[0] == "L":
                self.data.append(Laser(line))

    def iterator(self):
        """
        Just a normal iterator for the log data, with a twist for Odometry data.
        It also appends the previous odometry reading to the object so the
        motion model can do its thing.
        """
        for data in self.data:
            if isinstance(data, Odometry):
                data.prev_odometry = self.prev_odometry
                self.prev_odometry = data
            yield data


if __name__ == "__main__":
    l = Log('../data/log/robotdata1.log')

