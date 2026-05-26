# -*- coding: UTF-8 -*-

import numpy as np

from osgar.node import Node

from lib.lidar_interpreter import LidarInterpreter
from lib.lidar_to_pointcloud import LidarToPointCloud

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

        #self.interpreter = LidarInterpreter(output_frequency = 1.0)

        self.l2pc = None
        self.lidar_initialized = False

    def on_lidar_metadata(self, data):
        if not self.lidar_initialized:
            self.l2pc = LidarToPointCloud(data)
            self.lidar_initialized = True

    def on_lidar_scan3d(self, data):
        """
            Processes one Lidar distance scan.

            Args:
                data (numpy.array): lidar distance scan ... array of size H x W
        """
        if self.lidar_initialized:
            #print(self.time, self.interpreter.update(self.time, data))
            points = self.l2pc.convert(data)
            print("points:", points.shape)

    def on_lidar_reflectivity(self, data):
        """
            Processes one Lidar reflectivity scan.

            Args:
                data (numpy.array): lidar reflectivity scan ... array of size H x W
        """
        if self.lidar_initialized:
            #print("REFLECTIVITY:", data.shape)
            pass

    def draw(self):
        pass

