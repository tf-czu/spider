"""
    TODO
"""

from osgar.node import Node

import lib.least_squares_localization as loc

class LSqrLoc(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        # register a stream to be published
        bus.register('pose3d')
        # moving window in meters
        window = config.get('window', 5)
        # window that is used when the trajectory is supposed to be computed by
        # post-processing, that is, when all the data are gathered;
        # if `None` then no post-processed trajectory is computed
        post_window = config.get('post_window', None)
        # pruning of the data;
        # only every n-th data sample is taken when performing the
        # least-squares method
        prune = config.get('prune', 1)   
        # initial angle (in radians) and scale;
        # this is utilized to estimate pose3d at the start part of the
        # trajectory when the distance travelled is still shorter than the
        initial_angle  = config.get('initial_angle', -1.3)   
        initial_scale  = config.get('initial_scale',  1.0)   
        initial_window = config.get('initial_window', 1.0)   
        #
        self.localization = loc.LeastSquaresLocalization(
                                    window,
                                    post_window = post_window,
                                    prune = prune,
                                    initial_angle = initial_angle,
                                    initial_scale = initial_scale,
                                    initial_window = initial_window
                                )

    def on_nmea_data(self, data):
        self.localization.add_gps(self.time, data)

    def on_odom(self, data):
        self.localization.add_odo(self.time, data)

    def on_orientation(self, data):
        self.localization.add_ori(self.time, data)

    def draw(self):
        self.localization.draw()
