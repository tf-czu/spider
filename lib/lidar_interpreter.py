# -*- coding: UTF-8 -*-

import numpy as np

class LidarInterpreter:
    """
        Reads Lidar scans and returns: obstacle map, slope map.
    """

    def __init__(self, output_frequency = 1.0):
        self.output_frequency = output_frequency # [Hz]
        self.output_period = 1.0 / output_frequency # [s]
        self.last_output_time = None # [s]
        self.remembered_outputs = 0
        self.list_of_accumulated_scans = []

    def update(self, timestamp, scan):
        self.list_of_accumulated_scans.append(scan)
        t = timestamp.total_seconds()
        if self.last_output_time is None:
            self.last_output_time = t
            return False
        elif t - self.last_output_time >= self.output_period:
            self.process_accumulated_scans(timestamp)
            self.last_output_time = t
            return True
        else:
            return False

    def process_accumulated_scans(self, timestamp):
        print("PROCESSED")
