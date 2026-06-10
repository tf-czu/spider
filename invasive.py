"""
  TODO
"""

import math
from datetime import timedelta

import numpy as np

from osgar.node import Node
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.bus import BusShutdownException
from osgar.lib.route import Convertor as GPSConvertor
from osgar.lib import quaternion


class Invasive(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.4)
        self.waypoints = config.get('waypoints', [])
        self.start_geo_pose = None
        self.last_pose = None
        self.last_geo_pose = None
        self.gps_converter = None
        self.start_heading = None
        self.start_q_heading = None
        self.heading = None
        self.verbose = False

    def send_speed_cmd(self, speed, steering_angle):  # angle in radians
        if self.verbose:
            print(f"{self.time}, speed: {speed}, steering_angle: {math.degrees(steering_angle)}")
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )


    def on_bumpers(self, data):
        pass

    def on_pose2d(self, data):
        if data is not None:
            x, y, __ = data
            self.last_pose = [x/1000, y/1000]  # mm to m

    def on_pose3d(self, data):
        if self.start_heading is not None:
            [x, y, z], quat = data
            if self.start_q_heading is None:
                self.start_q_heading = -quaternion.heading(quat)  # IMU is probably inverted ?
                self.heading = self.start_heading
            else:
                q_heading = -quaternion.heading(quat)  # IMU is probably inverted ?
                self.heading = (q_heading - self.start_q_heading) - self.start_heading  # diff q_heading - initial gps_heading
                if self.verbose:
                    pass
                    # print(f"{self.time} Orientation - quaternion: {quat}, q_heading: {q_heading}, "
                    #      f"start heading (gps based): {self.start_heading}")

    def on_nmea_data(self, data):
        assert 'lat' in data, data
        assert 'lon' in data, data
        lat, lon = data['lat'], data['lon']
        if lat is not None and lon is not None:
           self.last_geo_pose = [lon, lat]

    def wait(self, duration):
        self.update()
        start_time = self.time
        while self.time - start_time < timedelta(seconds=duration):
            self.update()

    def go_straight(self, dist):
        print(self.time, 'Go straight')
        assert self.last_pose is not None
        start_pose = self.last_pose
        while True:
            if self.update() == 'pose2d':
                if math.hypot(start_pose[0] - self.last_pose[0],
                              start_pose[1] - self.last_pose[1]) < dist:
                    self.send_speed_cmd(self.max_speed, 0)
                else:
                    self.send_speed_cmd(0, 0)
                    break

    def get_geo_angle(self,start_geo_pose, geo_pose):
        lon_diff = geo_pose[0] - start_geo_pose[0]
        lat_diff = geo_pose[1] - start_geo_pose[1]
        return math.atan2(lat_diff, lon_diff)

    def dist2destination(self, waypoint):
        x0, y0 = self.gps_converter.geo2planar((self.last_geo_pose[0], self.last_geo_pose[1]))
        x, y = self.gps_converter.geo2planar((waypoint[0], waypoint[1]))
        return math.hypot(x0-x, y0-y)

    def navigate_to_waypoints(self, waypoint):
        print(f"Navigate to wp ({waypoint}), distance: {self.dist2destination(waypoint)}")
        while self.dist2destination(waypoint) > 1:
            if self.verbose:
                print("Dist: ", self.dist2destination(waypoint))
            if self.update() == 'pose2d' and self.heading is not None:
                heading_diff = normalizeAnglePIPI(self.heading - self.get_geo_angle(self.last_geo_pose, waypoint))  # radians
                if self.verbose:
                    print(f"Direction to wp: {self.get_geo_angle(self.last_geo_pose, waypoint)}, "
                          f"heading: {self.heading}, heading_diff: {heading_diff}")
                self.send_speed_cmd(self.max_speed, heading_diff)
        print(f"Waypoint {waypoint} reached.")

    def run(self):
        try:
            self.wait(1)
            assert self.last_pose is not None  # TODO add some initialization
            self.gps_converter = GPSConvertor((self.last_geo_pose[0], self.last_geo_pose[1]))  # define initial geo pose
            self.start_geo_pose = self.last_geo_pose
            self.go_straight(5)
            self.start_heading = self.get_geo_angle(self.start_geo_pose, self.last_geo_pose)
            for waypoint in self.waypoints:
                self.navigate_to_waypoints(waypoint)
        except BusShutdownException:
            pass
        self.send_speed_cmd(0, 0)


    def draw(self):
        # import matplotlib.pyplot as plt
        pass
