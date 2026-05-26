# -*- coding: UTF-8 -*-

import numpy as np
import ouster.sdk as ous

class PointCloudInterpreter:
    """
        Processes accumulated point clouds and returns maps:
            traversability map, obstacle map, slope map, ...
    """

    def __init__(self, output_frequency = 1.0):
        self.output_frequency = output_frequency # [Hz]
        self.output_period = 1.0 / output_frequency # [s]
        self.last_output_time = None # [datetime.timedelta]
        self.remembered_outputs = 0
        self.fifo_of_outputs = []
        self.list_of_accumulated_point_clouds = []

    def update(self, timestamp, points):
        self.list_of_accumulated_point_clouds.append(points)
        if self.last_output_time is None:
            self.last_output_time = timestamp
            return False
        elif (timestamp - self.last_output_time).total_seconds() >= self.output_period:
            self.process_accumulated_point_clouds(timestamp)
            self.last_output_time = timestamp
            return True
        else:
            return False

    def process_accumulated_point_clouds(self, timestamp):
        print("PROCESSING")
