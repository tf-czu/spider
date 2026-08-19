import json
import math
import os
import tempfile
import unittest
from datetime import timedelta
from unittest.mock import MagicMock

from osgar.bus import Bus
from osgar.lib.route import Convertor as GPSConvertor

from invasive_control.invasive import Invasive


def make_app(config=None):
    """Create an Invasive instance with a mock bus."""
    bus = Bus(MagicMock())
    app = Invasive(config=config or {}, bus=bus.handle('app'))
    return app


class TestLoadWaypoints(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix='.json')
        os.close(fd)
        self.addCleanup(os.unlink, self.path)

    def _write_json(self, data):
        with open(self.path, 'w', encoding='utf-8') as f:
            json.dump(data, f)

    def test_load_waypoints_from_file(self):
        self._write_json({'waypoints': [[14.0, 50.0], [14.1, 50.1]]})
        app = make_app()
        waypoints = app.load_waypoints(self.path)
        self.assertEqual(waypoints, [[14.0, 50.0], [14.1, 50.1]])

    def test_load_waypoints_missing_key(self):
        self._write_json({'other': []})
        app = make_app()
        with self.assertRaises(ValueError):
            app.load_waypoints(self.path)

    def test_load_waypoints_file_not_found(self):
        app = make_app()
        with self.assertRaises(FileNotFoundError):
            app.load_waypoints('/nonexistent/path.json')

    def test_waypoints_from_config_file(self):
        self._write_json({'waypoints': [[14.0, 50.0], [14.1, 50.1]]})
        app = make_app({'waypoints_file': self.path})
        self.assertEqual(app.waypoints, [[14.0, 50.0], [14.1, 50.1]])

    def test_waypoints_from_config_direct(self):
        # backward compatibility: waypoints directly in config
        app = make_app({'waypoints': [[14.0, 50.0]]})
        self.assertEqual(app.waypoints, [[14.0, 50.0]])

    def test_waypoints_empty_when_not_configured(self):
        app = make_app()
        self.assertEqual(app.waypoints, [])


class TestOnPose3d(unittest.TestCase):
    def test_stores_heading(self):
        app = make_app()
        app.on_pose3d([[1.0, 2.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
        self.assertIsNotNone(app.heading)

    def test_ignores_empty_data(self):
        app = make_app()
        app.on_pose3d(None)
        self.assertIsNone(app.heading)


class TestOnPose2d(unittest.TestCase):
    def test_stores_position_mm_to_m(self):
        app = make_app()
        app.on_pose2d([1000, 2000, 9000])
        self.assertEqual(app.last_pose, [1.0, 2.0])

    def test_ignores_empty_data(self):
        app = make_app()
        app.on_pose2d(None)
        self.assertIsNone(app.last_pose)


class TestGetMinObstacleDistance(unittest.TestCase):
    def _make_scan(self, distance_front, distance_side=0.0):
        """Create a 1024-ray scan with obstacle in front (index 512) and sides."""
        scan = [0.0] * 1024
        scan[512] = distance_front
        scan[0] = distance_side
        scan[1023] = distance_side
        return scan

    def test_no_scan_returns_none(self):
        app = make_app()
        app.last_scan = None
        self.assertIsNone(app.get_min_obstacle_distance())

    def test_front_obstacle(self):
        app = make_app()
        app.last_scan = self._make_scan(1500)
        self.assertEqual(app.get_min_obstacle_distance(), 1500)

    def test_ignores_side_obstacles(self):
        # obstacle on the side (index 0) should be outside FOV (120 deg)
        app = make_app()
        app.last_scan = self._make_scan(0.0, distance_side=500)
        self.assertIsNone(app.get_min_obstacle_distance())

    def test_ignores_zero(self):
        # all zeros -> no obstacle
        app = make_app()
        app.last_scan = [0.0] * 1024
        self.assertIsNone(app.get_min_obstacle_distance())

    def test_minimum_in_fov(self):
        # two obstacles in FOV: 1000 at front, smaller one nearby
        scan = [0.0] * 1024
        scan[512] = 2500
        scan[600] = 1200
        app = make_app()
        app.last_scan = scan
        self.assertEqual(app.get_min_obstacle_distance(), 1200)


class TestGoSafely(unittest.TestCase):
    def test_no_obstacle_sends_full_speed(self):
        app = make_app()
        app.last_scan = [0.0] * 1024  # no obstacle
        app.bus = MagicMock()
        app.go_safely(0.4, 0.0)
        # speed=0.4 -> round(400) -> 400 mm/s
        expected_speed = round(0.4 * 1000)
        self.assertGreater(app.bus.publish.call_args[0][1][0], 0)

    def test_obstacle_under_stop_dist(self):
        app = make_app()
        # obstacle at 1m (1000mm) -> should stop
        scan = [0.0] * 1024
        scan[512] = 1000
        app.last_scan = scan
        app.bus = MagicMock()
        app.go_safely(0.4, 0.0)
        published = app.bus.publish.call_args[0][1]
        self.assertEqual(published[0], 0)

    def test_obstacle_between_slow_and_stop(self):
        app = make_app()
        # obstacle at 2.5m (2500mm) -> should slow down
        scan = [0.0] * 1024
        scan[512] = 2500
        app.last_scan = scan
        app.bus = MagicMock()
        app.go_safely(0.4, 0.0)
        published = app.bus.publish.call_args[0][1]
        # speed should be between 0 and max (400 mm/s)
        self.assertGreater(published[0], 0)
        self.assertLess(published[0], round(0.4 * 1000))


class TestSelectTargetWaypoint(unittest.TestCase):
    def _setup_app(self, waypoints, pos=(14.0, 50.0)):
        """Create app with waypoints and GPS converter at given position."""
        app = make_app({'waypoints': waypoints})
        app.last_geo_pose = [pos[0], pos[1]]
        app.gps_converter = GPSConvertor(pos)
        return app

    def test_select_closest_from_window(self):
        app = self._setup_app([
            [14.0, 50.0],      # distance 0 -> closest
            [14.001, 50.001],
            [14.002, 50.002],
        ])
        idx, wp = app.select_target_waypoint()
        self.assertEqual(idx, 0)
        self.assertEqual(wp, [14.0, 50.0])

    def test_select_considers_only_window(self):
        # robot at origin; waypoint 0 far, later waypoints closer but outside window
        app = self._setup_app([
            [14.0, 50.0],      # closest (0)
            [14.001, 50.001],
            [14.002, 50.002],
            [14.003, 50.003],
            [14.004, 50.004],
            [14.005, 50.005],  # 6th - outside window
            [14.006, 50.006],
        ], pos=(14.0, 50.0))
        app.waypoint_index = 0
        # window is [0:5], all 5 are further than index 0
        idx, wp = app.select_target_waypoint()
        self.assertEqual(idx, 0)

    def test_advances_index_when_reached(self):
        app = self._setup_app([
            [14.0, 50.0],
            [14.001, 50.001],
            [14.002, 50.002],
        ])
        # robot is at waypoint 0 -> should be marked as reached, advance to 1
        app.waypoint_index = 0
        idx, wp = app.select_target_waypoint()
        self.assertEqual(idx, 0)


class TestOnNmeaData(unittest.TestCase):
    def test_stores_position_and_quality(self):
        app = make_app()
        app.on_nmea_data({'lat': 50.0, 'lon': 14.0, 'quality': 4})
        self.assertEqual(app.last_geo_pose, [14.0, 50.0])
        self.assertEqual(app.last_gps_quality, 4)

    def test_stores_quality_when_position_none(self):
        app = make_app()
        app.on_nmea_data({'lat': None, 'lon': None, 'quality': 0})
        self.assertEqual(app.last_gps_quality, 0)
        self.assertIsNone(app.last_geo_pose)

    def test_quality_defaults_to_none(self):
        app = make_app()
        app.on_nmea_data({'lat': 50.0, 'lon': 14.0})
        self.assertIsNone(app.last_gps_quality)


class TestCheckSensors(unittest.TestCase):
    def test_ok_when_all_sensors_available(self):
        app = make_app()
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = [14.0, 50.0]
        app.last_gps_quality = 1
        self.assertTrue(app.check_sensors())

    def test_missing_pose(self):
        app = make_app()
        app.last_pose = None
        app.last_geo_pose = [14.0, 50.0]
        app.last_gps_quality = 1
        self.assertFalse(app.check_sensors())

    def test_missing_gps(self):
        app = make_app()
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = None
        app.last_gps_quality = 1
        self.assertFalse(app.check_sensors())

    def test_low_quality(self):
        app = make_app()
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = [14.0, 50.0]
        app.last_gps_quality = 0
        self.assertFalse(app.check_sensors())

    def test_quality_none(self):
        app = make_app()
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = [14.0, 50.0]
        app.last_gps_quality = None
        self.assertFalse(app.check_sensors())

    def test_required_quality_from_config(self):
        app = make_app({'required_quality': 4})
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = [14.0, 50.0]
        app.last_gps_quality = 1
        self.assertFalse(app.check_sensors())

        app.last_gps_quality = 4
        self.assertTrue(app.check_sensors())


class TestWaitForSensors(unittest.TestCase):
    def test_success_when_sensors_available(self):
        app = make_app()
        app.time = timedelta(0)
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = [14.0, 50.0]
        app.last_gps_quality = 1
        self.assertTrue(app.wait_for_sensors(5))

    def test_timeout_when_sensors_never_arrive(self):
        app = make_app()
        app.time = timedelta(0)

        def fake_update():
            app.time += timedelta(seconds=1)

        app.update = fake_update
        self.assertFalse(app.wait_for_sensors(3))

    def test_success_after_wait(self):
        app = make_app()
        app.time = timedelta(0)

        calls = 0

        def fake_update():
            nonlocal calls
            calls += 1
            if calls >= 2:  # sensors arrive after first wait(1)
                app.last_pose = [0.0, 0.0]
                app.last_geo_pose = [14.0, 50.0]
                app.last_gps_quality = 1
            app.time += timedelta(seconds=1)

        app.update = fake_update
        self.assertTrue(app.wait_for_sensors(5))


class TestEnsureSensors(unittest.TestCase):
    def test_ok_when_sensors_available(self):
        app = make_app()
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = [14.0, 50.0]
        app.last_gps_quality = 1
        app.ensure_sensors()  # should not raise

    def test_pose_lost_raises(self):
        app = make_app()
        app.last_pose = None
        app.last_geo_pose = [14.0, 50.0]
        app.last_gps_quality = 1
        with self.assertRaises(RuntimeError):
            app.ensure_sensors()

    def test_gps_lost_stops_and_waits(self):
        app = make_app()
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = None
        app.last_gps_quality = None
        app.time = timedelta(0)

        # GPS recovers after a short wait
        calls = 0

        def fake_update():
            nonlocal calls
            calls += 1
            if calls >= 2:
                app.last_geo_pose = [14.0, 50.0]
                app.last_gps_quality = 1
            app.time += timedelta(seconds=1)

        app.update = fake_update
        app.ensure_sensors()  # should stop, wait, and recover without raising
        self.assertEqual(app.last_geo_pose, [14.0, 50.0])

    def test_gps_not_recovered_raises(self):
        app = make_app()
        app.last_pose = [0.0, 0.0]
        app.last_geo_pose = None
        app.last_gps_quality = None
        app.time = timedelta(0)

        def fake_update():
            app.time += timedelta(seconds=1)

        app.update = fake_update
        with self.assertRaises(RuntimeError):
            app.ensure_sensors()


class TestGetGeoAngle(unittest.TestCase):
    def test_east(self):
        angle = Invasive.get_geo_angle([14.0, 50.0], [14.001, 50.0])
        self.assertAlmostEqual(angle, 0.0, places=5)

    def test_north(self):
        angle = Invasive.get_geo_angle([14.0, 50.0], [14.0, 50.001])
        self.assertAlmostEqual(angle, math.pi / 2, places=5)

    def test_west(self):
        angle = Invasive.get_geo_angle([14.0, 50.0], [13.999, 50.0])
        self.assertAlmostEqual(angle, math.pi, places=5)


class TestDist2Destination(unittest.TestCase):
    def test_distance(self):
        app = make_app()
        app.last_geo_pose = [14.0, 50.0]
        app.gps_converter = GPSConvertor((14.0, 50.0))
        # waypoint ~111m east (1 deg lon ~ 71.7km at 50N, but use converter)
        dist = app.dist2destination([14.001, 50.0])
        self.assertGreater(dist, 0)
        self.assertLess(dist, 200)

    def test_zero_distance(self):
        app = make_app()
        app.last_geo_pose = [14.0, 50.0]
        app.gps_converter = GPSConvertor((14.0, 50.0))
        dist = app.dist2destination([14.0, 50.0])
        self.assertAlmostEqual(dist, 0.0, places=5)


if __name__ == '__main__':
    unittest.main()