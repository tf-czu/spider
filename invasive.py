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


def geo_length(pos1, pos2):
    "return distance on sphere for two integer positions in milliseconds"
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    scale = 40000000/(360*3600000)
    return math.hypot((pos2[0] - pos1[0])*x_scale, pos2[1] - pos1[1]) * scale

def geo_angle(pos1, pos2):
    if geo_length(pos1, pos2) < 1.0:
        return None
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    return math.atan2(pos2[1] - pos1[1], (pos2[0] - pos1[0])*x_scale)


class Invasive(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'scan')
        self.max_speed = config.get('max_speed', 0.2)
        self.waypoints = config.get('waypoints', [])
        self.start_pose = None
        self.last_pose = None
        self.last_geo_pose = None
        self.gps_converter = None
        self.start_heading = None
        self.heading = None
        self.verbose = False

    def send_speed_cmd(self, speed, steering_angle):  # angle in radians
        if self.verbose:
            print("steering_angle: ", math.degrees(steering_angle))
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )


    def on_bumpers(self, data):
        pass

    def on_pose2d(self, data):
       self.last_pose = data

    def on_pose3d(self, data):
        if self.start_heading is not None:
            [x, y, z], quat = data
            q_heading = quaternion.heading(quat)
            self.heading = q_heading - self.start_heading

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
        self.start_pose = self.last_pose
        while True:
            if self.update() == 'pose2d':
                if math.hypot(self.start_pose[0] - self.last_pose[0],
                              self.start_pose[1] - self.last_pose[1]) < dist:
                    self.send_speed_cmd(self.max_speed, 0)
                else:
                    self.send_speed_cmd(0, 0)
                    break

    def get_heading(self, geo_pose):
        x, y = self.gps_converter.geo2planar((geo_pose[0], geo_pose[1]))
        return math.atan2(x, y)

    def dist2destination(self, waypoint):
        x0, y0 = self.gps_converter.geo2planar((self.last_geo_pose[0], self.last_geo_pose[1]))
        x, y = self.gps_converter.geo2planar((waypoint[0], waypoint[1]))
        return math.hypot(x0-x, y0-y)

    def navigate_to_waypoints(self, waypoint):
        print(self.dist2destination(waypoint))
        while self.dist2destination(waypoint) > 1:
            if self.verbose:
                print("Dist: ", self.dist2destination(waypoint))
            if self.update() == 'pose2d' and self.heading is not None:
                heading_diff = self.heading - self.get_heading(waypoint)  # radians
                self.send_speed_cmd(self.max_speed, heading_diff)
        print(f"Waypoint {waypoint} reached.")

    def run(self):
        try:
            self.wait(1)
            self.gps_converter = GPSConvertor((self.last_geo_pose[0], self.last_geo_pose[1]))  # define initial geo pose
            self.go_straight(5)
            self.start_heading = self.get_heading((self.last_geo_pose[0], self.last_geo_pose[1]))
            for waypoint in self.waypoints:
                self.navigate_to_waypoints(waypoint)
        except BusShutdownException:
            pass


    def draw(self):
        # import matplotlib.pyplot as plt
        pass

# vim: expandtab sw=4 ts=4