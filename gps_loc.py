"""
    TODO
"""
from tabnanny import verbose
# from lib.localization import Localization
from unittest.mock import MagicMock as Localization, MagicMock  # Real Localization is not working yet
Localization.update_xyz_gps = MagicMock()
Localization.get_pose3d = MagicMock(return_value=(1, 1, 1))

from osgar.node import Node
from osgar.lib.route import Convertor

def list2xy(data):
    x = [coord[0] for coord in data]
    y = [coord[1] for coord in data]
    return x, y

class GpsLocalization(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose3d')  # register a stream to be published
        self.localization = Localization()
        self.con = None
        self.alt_0 = None

        self.verbose = False
        self.debug_org_position = []
        self.debug_position = []

    def on_nmea_data(self, data):
        lon = data["lon"]
        lat = data["lat"]
        assert data["lon_dir"] == "E"
        assert data["lat_dir"] == "N"
        alt = data["alt"]
        if self.alt_0 is not None:
            z = alt - self.alt_0
        else:
            self.alt_0 = alt
            z = 0
        if self.con:
            x, y = self.con.geo2planar((lon, lat))
            Localization.update_xyz_gps((x, y, z), qps_err = (0.05, 0.05, 0.15))
            if self.verbose:
                self.debug_org_position.append([x, y])
        else:
            self.con = Convertor((lon, lat))


    def on_timer(self, data):
        pose3d = Localization.get_pose3d(self.time)
        if pose3d:
            if verbose:
                (x, y, __), __ = pose3d
                self.debug_position.append([x, y])
            self.publish("pose3d", pose3d)

    def draw(self):
        # in verbose mode and with --draw parameter: draw a plot
        import matplotlib.pyplot as plt
        x, y = list2xy(self.debug_org_position)
        plt.plot(x, y, "k.-", label="org_gps")
        x, y = list2xy(self.debug_position)
        plt.plot(x, y, "r.-", label="kalman")
        plt.legend()
        plt.show()
