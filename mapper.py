# -*- coding: UTF-8 -*-

import numpy as np

from osgar.node import Node

from lib.lidar_interpreter import LidarInterpreter

class Mapper(Node):
    """
        Reads Lidar data and returns: obstacle map, slope map.
    """

    def __init__(self, config, bus):
        super().__init__(config, bus)

        # register a stream to be published
        #bus.register('pose3d')

        # for debugging
        self.verbose = False # super-class Node sets this to `True` if --verbose parameter is applied

        self.interpreter = LidarInterpreter(output_frequency = 1.0)

    def on_lidar_scan3d(self, data):
        """
            Process one Lidar scan.

            Args:
                data (numpy.array): 2D array containing a lidar scan;
                    our usual size is 32 x 1024
        """
        print(self.time, self.interpreter.update(self.time, data))

    def draw(self):
        pass

