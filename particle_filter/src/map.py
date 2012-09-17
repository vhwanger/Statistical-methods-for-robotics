"""
This module handles all map information that comes from the wean.dat file.
"""
import re
import numpy as np
import Image
import pdb
from decimal import Decimal 
D = Decimal

DEBUG = True
MAX_DISTANCE = 8183 # in centimeters

class ParameterException(Exception): pass
class MapDataException(Exception): pass

class Map:
    def __init__(self, filename):
        f = open(filename, 'r')
        self.parameters = {}
        self.map = []

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
        for line in lines:
            if line.strip():
                if re.compile('[a-zA-Z]').findall(line):
                    continue
                points = [D(p) for p in line.split()]
                if len(points) != self.num_columns:
                    print line
                    raise MapDataException("Line has incorrect number of data "
                                           "points. Should have %s, has %s" %
                                           (self.num_columns, len(points)))
                self.map.append(points)

    def expected_distance(self, x, y, theta):
        pass

if __name__ == '__main__':
    wean_map = Map('../data/map/wean.dat')
    np_array = np.array(wean_map.map)
    np_array *= 255
    np_array = np_array.clip(min=0)
    img = Image.fromarray(np.transpose(np.uint8(np_array)))
    img.convert('RGB').save("test.png")
    
