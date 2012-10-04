"""
This module handles all map information that comes from the wean.dat file.
"""
import re
import math
import pdb
import numpy as np
import Image
from constants import (LIDAR_ANGLE_INTERVAL, MAX_DISTANCE_CM, INIT_THRESHOLD)
from decimal import Decimal 
D = Decimal

DEBUG = True

class ParameterException(Exception): pass
class MapDataException(Exception): pass

class Map:
    def __init__(self, filename):
        f = open(filename, 'r')
        self.parameters = {}

        lines = f.readlines()
        self.read_map_parameters(lines)
        self.read_map_points(lines)

        f.close()
    
    def read_map_parameters(self, lines):
        """
        Checks the first 8 lines for parameters and stores their value into
        self.parameters. Expects the parameters listed below.

        Also reads in the dimensions of the map and stores it into self.num_rows
        and self.num_columns
        """
        parameters = ['global_mapsize_x', 
                      'global_mapsize_y', 
                      'resolution',
                      'autoshifted_x', 
                      'autoshifted_y']

        parameter_lines = [p for p in lines[0:10] if "robot_specifications" in p]

        # read in the parameter name after ->
        for line in parameter_lines:
            try:
                parameter_name = line.split()[0].split()[0].split('>')[1]
                value = int(line.split()[-1])
                self.parameters[parameter_name] = value
                if DEBUG:
                    print "Parameter %s set to %s" % (parameter_name, value)
            except IndexError:
                raise ParameterException("Malformed robot_specifications")
        
        # check that all the parameters are there
        for param in parameters:
            if param not in self.parameters:
                raise ParameterException("Missing required parameter %s" %
                                         param)

        # get the map size
        try:
            global_map = [p for p in lines[0:12] if "global_map[0]" in p][0]
            self.num_rows, self.num_columns = [int(p) for p in
                                               global_map.split()[1:3]]
            self.map = np.empty((self.num_rows, self.num_columns))
            if DEBUG:
                print "# map rows: %s\n# map columns: %s" % (self.num_rows,
                                                              self.num_columns)
        except IndexError:
            raise ParameterException("Couldn't find the global_map[0] parameter")

    def read_map_points(self, lines):
        """
        Reads in the probabilities from the map and stores it in a two
        dimenional array in self.map
        """
        search = re.compile('[a-zA-Z]')
        row_counter = 0
        for (num_line, line) in enumerate(lines):
            if line.strip():
                if search.findall(line):
                    continue
                points = np.array(map(float, line.split()))
                if len(points) != self.num_columns:
                    print line
                    raise MapDataException("Line has incorrect number of data "
                                           "points. Should have %s, has %s" %
                                           (self.num_columns, len(points)))
                self.map[row_counter] = points
                row_counter += 1

    def is_free(self, coord):
        resolution = self.parameters['resolution']
        try:
            if coord[0] > 8000 or coord[1] > 8000:
                return False
            if self.map[coord[0] / resolution][coord[1] / resolution] < INIT_THRESHOLD:
                return False
            else:
                return True
        except IndexError:
            raise Exception("IndexError %s %s" % (coord[0] / resolution,
                                                  coord[1] / resolution))

    def open_cells(self):
        """
        Returns all locations where the robot could be based on the
        probabilities in the map.
        """
        points = np.where(self.map > INIT_THRESHOLD)
        return zip(points[0] * self.parameters['resolution'], 
                   points[1] * self.parameters['resolution'])

    def ray_trace(self, x, y, all_coords):
        """
        This takes in the location of the robot and a list of rays. The rays
        variable is organized as follows:

            [
                ([(x1, y1), (x2, y2), ...], theta_1), 
                ([(x1, y1), (x2, y2), ...], theta_2), 
                ...
            ]    

        The theta isn't used for anything in this function, it was just a handy
        placeholder to tell me which ray the list of coordinates corresponded
        to.

        For each set of ray points, it calculates the distance from x, y. Once
        we hit a point where the probability is <=0, we call that the final
        distance.
        """
        ray_distances = []
        resolution = self.parameters['resolution']
        for i in range(all_coords.shape[0]):
            distance = 0
            
            # filter x and y coords to stay within map regions
            #ray_range = np.bitwise_and(x_coords[i]<799,y_coords[i]<799)

            # determine ending index where the ray stops
            #len_ray = ray_range[ray_range==True].shape[0]

            # zip up the x and y coords
            #ray_coords = np.c_[x_coords[i,0:len_ray], y_coords[i,0:len_ray]]
            ray_coords = all_coords[i,(all_coords[i,:,0] < 799) & (all_coords[i,:,1] < 799),:]

            # look up all the coordinates in the map and find where the map is
            # less than or equal to zero (this is a wall)
            ray_values, = np.where(self.map[ray_coords[:,0], ray_coords[:,1]] <= .5)

            # some special exceptions
            if not ray_values.shape[0]:
                if not len(ray_coords):
                    end_of_ray = np.array([x/resolution, y/resolution])
                else:
                    end_of_ray = ray_coords[len(ray_values)]
            else:
                # get the end of the ray
                end_of_ray = ray_coords[ray_values.item(0)]

            # find the distance from the originating point
            distance = (math.sqrt((end_of_ray.item(0) - x/resolution)**2 +
                                  (end_of_ray.item(1) - y/resolution)**2) *
                        resolution)

            ray_distances.append(distance)
        return ray_distances

    def expected_distance(self, x, y, theta):
        """
        This does ray-tracing for determining the expected distance of the lidar
        at any given point. The lidar has 180 beams that come out of it, and for
        each beam, we need to decide (given some hypothetical coordinate) what
        the expected distance is for each of those beams. We know the max range
        of the lidar is 8183cm and our map's resolution is 10cm per box. So for
        some angle theta, we need to look up if there is a wall at 
        (15cm * cos(theta), 15cm * sin(theta))
        (25cm * cos(theta), 25cm * sin(theta)))
        ...
        (8185cm * cos(theta), 8185cm * sin(theta))

        Because we need to do this for each theta we have, I implement this as
        an outer product

        [sin(theta_1),      * [ 5, 15, 25, ... 8185]
         sin(theta_2),
         ...
         sin(theta_180)]

        Then we floor these values to get it into real pixel coordinates, and
        then we do a lookup against the table to determine when a particular ray
        hits a wall (self.ray_trace).
        """
        distances = None
        x_coords = None
        y_coords = None
        resolution = self.parameters['resolution']
        distance_steps = np.arange(5, MAX_DISTANCE_CM/resolution);
        angles = np.arange(1, 180, LIDAR_ANGLE_INTERVAL) + math.degrees(theta)-90

        def calc_cos(deg):
            return math.cos(math.radians(deg))

        def calc_sin(deg):
            return math.sin(math.radians(deg))


        cosvfunc = np.vectorize(calc_cos)
        cos_angles = cosvfunc(angles)

        sinvfunc = np.vectorize(calc_sin)
        sin_angles = sinvfunc(angles)

        x_coords = np.outer(cos_angles, distance_steps) + x / resolution
        y_coords = np.outer(sin_angles, distance_steps) + y / resolution
        x_coords = x_coords.astype(int)
        y_coords = y_coords.astype(int)
        
        all_coords = np.dstack((x_coords, y_coords))
        distances = self.ray_trace(x, y, all_coords)
        return (x_coords, y_coords, distances)


if __name__ == '__main__':
    wean_map = Map('../data/map/wean.dat')
    np_array = np.array(wean_map.map)
    np_array *= 255
    np_array = np_array.clip(min=0)
    img = Image.fromarray(np.transpose(np.uint8(np_array)))
    img.convert('RGB').save("map.png")


