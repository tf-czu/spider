# -*- coding: UTF-8 -*-

import numpy as np

from osgar.node import Node

from lib.pointcloud_interpreter import PointCloudInterpreter
from lib.lidar_to_pointcloud import LidarToPointCloud


class Mapper(Node):
    """
    OSGAR node responsible for lidar-based terrain mapping.

    The Mapper receives raw lidar data from the OSGAR bus,
    converts the scans into point clouds and forwards them
    to PointCloudInterpreter for higher-level terrain analysis.

    Input streams:
        - lidar_metadata
        - lidar_scan3d
        - lidar_reflectivity

    Internal processing pipeline:

        lidar scan
            |
        LidarToPointCloud
            |
        point cloud (N x 3)
            |
        PointCloudInterpreter
            |
        terrain maps

    Expected output maps:
        - obstacle map
        - slope map
        - traversability map
        - height map
    """

    def __init__(self, config, bus):
        """
        Initializes the Mapper node.

        Args:
            config (dict):
                OSGAR node configuration.

            bus:
                OSGAR communication bus.
        """

        super().__init__(config, bus)

        # register a stream to be published
        # bus.register('pose3d')

        # super-class Node sets this to `True` if --verbose parameter is applied
        self.verbose = False

        # interprets accumulated point clouds into terrain maps
        self.interpreter = PointCloudInterpreter(output_frequency = 1.0)

        # lidar scan -> point cloud converter
        # initialized after lidar metadata are received
        self.l2pc = None

    def on_lidar_metadata(self, data):
        """
        Processes lidar geometry metadata.

        This callback is expected to be processed before
        lidar scans are received.

        The metadata are used to initialize:
            - sensor geometry
            - XYZ lookup table
            - lidar scan to point cloud conversion

        Args:
            data (str):
                JSON string containing lidar metadata.
        """

        # initialize lidar converter only once
        if self.l2pc is None:
            self.l2pc = LidarToPointCloud(data)

    def on_lidar_scan3d(self, data):
        """
        Processes one lidar distance scan.

        The incoming range image is converted into
        a 3D point cloud and passed to PointCloudInterpreter.

        Args:
            data (numpy.array):
                Lidar range scan of size H x W.
        """

        # ignore scans until lidar metadata are initialized
        if self.l2pc is not None:

            # convert lidar scan to Nx3 point cloud
            points = self.l2pc.convert(data)

            # update terrain interpretation pipeline
            output = self.interpreter.update(self.time, points)
            print("output:", output)

    def on_lidar_reflectivity(self, data):
        """
        Processes one lidar reflectivity scan.

        Reflectivity information may later be used for:
            - vegetation detection
            - terrain classification
            - surface material estimation

        Args:
            data (numpy.array):
                Lidar reflectivity scan of size H x W.
        """

        if self.l2pc is not None:
            # print("REFLECTIVITY:", data.shape)
            pass

    def draw(self):
        pass
