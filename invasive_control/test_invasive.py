import math
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


class TestOnPose3d(unittest.TestCase):
    def test_stores_position(self):
        app = make_app()
        app.on_pose3d([[1.0, 2.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
        self.assertEqual(app.last_pose, [1.0, 2.0])

    def test_ignores_empty_data(self):
        app = make_app()
        app.on_pose3d(None)
        self.assertIsNone(app.last_pose)


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