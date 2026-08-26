# -*- coding: UTF-8 -*-

import unittest
from unittest.mock import MagicMock

from lib.tracker import Tracker

class TestTracker(unittest.TestCase):

    def test_abstract_class(self):
        with self.assertRaises(TypeError):
            t = Tracker()

if __name__ == '__main__':
    unittest.main()

