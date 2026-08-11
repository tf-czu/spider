import os
import tempfile
import unittest

from tools.load_gps_path import load_segments, nmea_to_decimal, parse_nmea_time


class TestNmeaToDecimal(unittest.TestCase):
    def test_north_east(self):
        self.assertAlmostEqual(nmea_to_decimal('5005.1234', 'N'), 50.08539, places=5)
        self.assertAlmostEqual(nmea_to_decimal('01430.5678', 'E'), 14.50946, places=5)

    def test_south_west(self):
        self.assertAlmostEqual(nmea_to_decimal('5005.1234', 'S'), -50.08539, places=5)
        self.assertAlmostEqual(nmea_to_decimal('01430.5678', 'W'), -14.50946, places=5)

    def test_empty_value(self):
        self.assertIsNone(nmea_to_decimal('', 'N'))
        self.assertIsNone(nmea_to_decimal(None, 'N'))


class TestParseNmeaTime(unittest.TestCase):
    def test_without_fraction(self):
        dt = parse_nmea_time('123456')
        self.assertIsNotNone(dt)
        self.assertEqual(dt.hour, 12)
        self.assertEqual(dt.minute, 34)
        self.assertEqual(dt.second, 56)

    def test_with_fraction(self):
        dt = parse_nmea_time('123456.789')
        self.assertIsNotNone(dt)
        self.assertEqual(dt.hour, 12)
        self.assertEqual(dt.minute, 34)
        self.assertEqual(dt.second, 56)
        self.assertEqual(dt.microsecond, 789000)

    def test_empty_value(self):
        self.assertIsNone(parse_nmea_time(''))
        self.assertIsNone(parse_nmea_time(None))

    def test_invalid_value(self):
        self.assertIsNone(parse_nmea_time('not-a-time'))


class TestLoadSegments(unittest.TestCase):
    """Helper to write NMEA content to a temp file and call load_segments."""

    def _write_temp_file(self, content):
        fd, path = tempfile.mkstemp(suffix='.nmea')
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(content)
        self.addCleanup(os.unlink, path)
        return path

    def _gga_line(self, time_str, lat, lat_dir, lon, lon_dir, quality):
        return f'$GPGGA,{time_str},{lat},{lat_dir},{lon},{lon_dir},{quality},08,0.9,545.4,M,46.9,M,,*47\n'

    def test_filters_by_quality_default(self):
        content = (
            self._gga_line('100000.00', '5005.1234', 'N', '01430.5678', 'E', '1') +
            self._gga_line('100001.00', '5005.2234', 'N', '01430.6678', 'E', '2') +
            self._gga_line('100002.00', '5005.3234', 'N', '01430.7678', 'E', '1')
        )
        path = self._write_temp_file(content)
        segments = load_segments(path)

        self.assertEqual(len(segments), 1)
        self.assertEqual(len(segments[0]), 2)
        # Only quality=1 points should be present
        self.assertAlmostEqual(segments[0][0][0], 50.08539, places=5)
        self.assertAlmostEqual(segments[0][1][0], 50.08872, places=5)

    def test_filters_by_quality_2(self):
        content = (
            self._gga_line('100000.00', '5005.1234', 'N', '01430.5678', 'E', '1') +
            self._gga_line('100001.00', '5005.2234', 'N', '01430.6678', 'E', '2') +
            self._gga_line('100002.00', '5005.3234', 'N', '01430.7678', 'E', '4')
        )
        path = self._write_temp_file(content)
        segments = load_segments(path, quality=2)

        self.assertEqual(len(segments), 1)
        self.assertEqual(len(segments[0]), 1)
        self.assertAlmostEqual(segments[0][0][0], 50.08705666666667, places=5)

    def test_filters_by_quality_4(self):
        content = (
            self._gga_line('100000.00', '5005.1234', 'N', '01430.5678', 'E', '1') +
            self._gga_line('100001.00', '5005.2234', 'N', '01430.6678', 'E', '2') +
            self._gga_line('100002.00', '5005.3234', 'N', '01430.7678', 'E', '4')
        )
        path = self._write_temp_file(content)
        segments = load_segments(path, quality=4)

        self.assertEqual(len(segments), 1)
        self.assertEqual(len(segments[0]), 1)
        self.assertAlmostEqual(segments[0][0][0], 50.08872, places=5)

    def test_no_matching_quality_returns_empty(self):
        content = (
            self._gga_line('100000.00', '5005.1234', 'N', '01430.5678', 'E', '1') +
            self._gga_line('100001.00', '5005.2234', 'N', '01430.6678', 'E', '2')
        )
        path = self._write_temp_file(content)
        segments = load_segments(path, quality=4)

        self.assertEqual(segments, [])

    def test_segment_split_on_time_gap(self):
        content = (
            self._gga_line('100000.00', '5005.1234', 'N', '01430.5678', 'E', '1') +
            self._gga_line('100001.00', '5005.2234', 'N', '01430.6678', 'E', '1') +
            self._gga_line('100010.00', '5005.3234', 'N', '01430.7678', 'E', '1')
        )
        path = self._write_temp_file(content)
        segments = load_segments(path, threshold_seconds=5.0)

        self.assertEqual(len(segments), 2)
        self.assertEqual(len(segments[0]), 2)
        self.assertEqual(len(segments[1]), 1)

    def test_ignores_non_gpgga_lines(self):
        content = (
            '$GPRMC,100000.00,A,5005.1234,N,01430.5678,E,0.0,0.0,010100,,,D*6A\n' +
            self._gga_line('100000.00', '5005.1234', 'N', '01430.5678', 'E', '1') +
            '$GPGSA,A,3,01,02,03,,,,\n'
        )
        path = self._write_temp_file(content)
        segments = load_segments(path)

        self.assertEqual(len(segments), 1)
        self.assertEqual(len(segments[0]), 1)

    def test_ignores_malformed_lines(self):
        content = (
            '$GPGGA,100000.00,5005.1234,N,01430.5678,E,1,08,0.9,545.4,M,46.9,M,,*47\n' +
            '$GPGGA,100001.00,,N,01430.6678,E,1,08,0.9,545.4,M,46.9,M,,*47\n' +
            '$GPGGA,100002.00,5005.3234,N,,E,1,08,0.9,545.4,M,46.9,M,,*47\n' +
            '$GPGGA,100003.00,5005.4234,N,01430.8678,E,1,08,0.9,545.4,M,46.9,M,,*47\n'
        )
        path = self._write_temp_file(content)
        segments = load_segments(path)

        self.assertEqual(len(segments), 1)
        self.assertEqual(len(segments[0]), 2)


if __name__ == '__main__':
    unittest.main()
