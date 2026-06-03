# -*- coding: UTF-8 -*-

import time

import numpy as np

from lib.terrain_mapper import TerrainMapper
from lib.angular_mapper import AngularMapper
from lib.hole_detector_height import HoleDetectorHeight


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
        - angular obstacle map

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
        self.output_frequency = output_frequency
        self.output_period = 1.0 / output_frequency
        self.last_output_time = None

        self.remembered_outputs = 3
        self.fifo_of_outputs = []
        self.list_of_accumulated_point_clouds = []

        self.terrain_mapper = TerrainMapper()
        #self.angular_mapper = AngularMapper(obstacle_detection_method = "nearest_point")
        self.angular_mapper = AngularMapper(obstacle_detection_method = "nearest_cluster")
        #self.hole_detector = HoleDetectorHeight(hole_detection_method = "mean")
        self.hole_detector = HoleDetectorHeight(hole_detection_method = "percentile")

        # measuring time performance
        self.measure_execution_time = True
        self.execution_times = []
        self.terrain_mapper_execution_times = []
        self.angular_mapper_execution_times = []
        self.hole_detector_execution_times = []

    def update(self, timestamp, points):
        """
        Adds one point cloud to the internal accumulation buffer.

        Once enough time has elapsed since the previous output,
        the accumulated point clouds are processed into output maps.

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the incoming point cloud.

            points (numpy.array):
                Point cloud of shape N x 3.

        Returns:
            dict:
                Output dictionary if new maps were generated.

            None:
                If not enough time has elapsed yet.
        """
        self.list_of_accumulated_point_clouds.append(points)

        if self.last_output_time is None:
            self.last_output_time = timestamp
            return None

        if (timestamp - self.last_output_time).total_seconds() >= self.output_period:
            t0 = time.perf_counter()
            output = self.process_accumulated_point_clouds(timestamp)
            if self.measure_execution_time:
                self.execution_times.append(time.perf_counter() - t0)
                #if len(self.execution_times) % 5 == 0:
                #     print("execution time [ms]: avg = {:.1f}, min = {:.1f}, max = {:.1f}".format(
                #        1000*np.mean(self.execution_times),
                #        1000*np.min(self.execution_times),
                #        1000*np.max(self.execution_times)))
            self.last_output_time = timestamp
            return output

        return None

    def get_execution_times(self):
        result = ""
        #if self.measure_execution_time:
        #    result += "avg: {:.1f}".format(1000*np.mean(self.execution_times))
        #    result += " min: {:.1f}".format(1000*np.min(self.execution_times))
        #    result += " max: {:.1f}".format(1000*np.max(self.execution_times))
        if self.measure_execution_time:
            result += "tot: {:.1f}".format(1000*np.mean(self.execution_times))
            result += " ter: {:.1f}".format(1000*np.min(self.terrain_mapper_execution_times))
            result += " ang: {:.1f}".format(1000*np.max(self.angular_mapper_execution_times))
            result += " hol: {:.1f}".format(1000*np.max(self.hole_detector_execution_times))
        return result

    def process_accumulated_point_clouds(self, timestamp):
        """
        Processes accumulated point clouds and generates output maps.

        This method merges all accumulated point clouds and passes them
        to specialized mappers responsible for different output
        representations.

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the generated output maps.

        Returns:
            dict:
                Dictionary containing output maps and timestamp.
        """
        if not self.list_of_accumulated_point_clouds:
            return None

        points = np.vstack(self.list_of_accumulated_point_clouds)

        t0 = time.perf_counter()
        terrain_output = self.terrain_mapper.compute(points)
        if self.measure_execution_time:
            self.terrain_mapper_execution_times.append(time.perf_counter() - t0)

        t0 = time.perf_counter()
        angular_output = self.angular_mapper.compute(points)
        if self.measure_execution_time:
            self.angular_mapper_execution_times.append(time.perf_counter() - t0)

        t0 = time.perf_counter()
        hole_output = self.hole_detector.compute(terrain_output["min_map"])
        if self.measure_execution_time:
            self.hole_detector_execution_times.append(time.perf_counter() - t0)

        output = {
            "timestamp": timestamp,
        }
        output.update(terrain_output)
        output.update(angular_output)
        output.update(hole_output)

        self.fifo_of_outputs.append(output)

        while len(self.fifo_of_outputs) > self.remembered_outputs:
            self.fifo_of_outputs.pop(0)

        self.list_of_accumulated_point_clouds.clear()

        return output
