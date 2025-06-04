# -*- coding: UTF-8 -*-

import unittest
from unittest.mock import MagicMock

import tracker

class TestTracker(unittest.TestCase):

    def test_abstract_class(self):
        with self.assertRaises(TypeError):
            t = tracker.Tracker()

if __name__ == '__main__':
    unittest.main()

