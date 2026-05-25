# -*- coding: UTF-8 -*-

import numpy as np

from osgar.node import Node

class LidarInterpreter(Node):
    """
        Reads Lidar data and returns: obstacle map, slope map.
    """

    def __init__(self, config, bus):
        super().__init__(config, bus)

        # register a stream to be published
        #bus.register('pose3d')

        # for debugging
        self.verbose = False # super-class Node sets this to `True` if --verbose parameter is applied

    def on_lidar_scan3d(self, data):
        """
            Process one Lidar scan.

            Args:
                data (list of list of int): array of size 32 x 1024
        """
        print(self.time, np.array(data).shape)

    def draw(self):
        pass

