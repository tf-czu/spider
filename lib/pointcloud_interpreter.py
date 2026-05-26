# -*- coding: UTF-8 -*-

import numpy as np


class PointCloudInterpreter:
    """
    Interprets incoming point clouds and produces terrain maps.

    The interpreter accumulates multiple point clouds over time and
    periodically processes them into higher-level representations such as:

        - traversability map
        - obstacle map
        - slope map
        - height map
        - percentile-based terrain maps

    Input:
        point cloud of shape N x 3

    Output:
        terrain-related maps computed at a lower frequency than the
        incoming lidar scans.
    """

    def __init__(self, output_frequency = 1.0):
        """
        Args:
            output_frequency (float):
                Frequency [Hz] at which output maps are generated.

                Example:
                    lidar input frequency  = 10 Hz
                    map output frequency   = 1 Hz

                In this case, approximately 10 incoming point clouds
                are accumulated before one terrain map update.
        """

        # desired output frequency [Hz]
        self.output_frequency = output_frequency

        # desired output period [s]
        self.output_period = 1.0 / output_frequency

        # timestamp of the last generated output map
        self.last_output_time = None  # [datetime.timedelta]

        # number of remembered outputs
        self.remembered_outputs = 0

        # FIFO storage for previously generated outputs
        self.fifo_of_outputs = []

        # accumulated point clouds waiting for processing
        self.list_of_accumulated_point_clouds = []

    def update(self, timestamp, points):
        """
        Adds one point cloud to the internal accumulation buffer.

        Once enough time has elapsed since the previous output,
        the accumulated point clouds are processed into terrain maps.

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the incoming point cloud.

            points (numpy.array):
                Point cloud of shape N x 3.

        Returns:
            bool:
                True  -> new maps were generated
                False -> not enough time has elapsed yet
        """

        # store incoming point cloud
        self.list_of_accumulated_point_clouds.append(points)

        # initialize output timer
        if self.last_output_time is None:
            self.last_output_time = timestamp
            return False

        # process accumulated data periodically
        elif (timestamp - self.last_output_time).total_seconds() >= self.output_period:

            self.process_accumulated_point_clouds(timestamp)

            self.last_output_time = timestamp

            return True

        else:
            return False

    def process_accumulated_point_clouds(self, timestamp):
        """
        Processes accumulated point clouds and generates terrain maps.

        This method is expected to:
            - merge accumulated point clouds
            - compute height maps
            - compute percentile-based terrain statistics
            - compute slope maps
            - detect obstacles
            - generate traversability maps

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the generated output maps.
        """

        print("PROCESSING")

