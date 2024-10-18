import numpy as np
import osgar.lib.quaternion as quaternion
from lib.kalman import KalmanFilterLocalization

class Localization:
    def __init__(self, remember_history = False):
        """
            Args:
                remember_history (bool): if `True`, input values will be stored
                    to `history` attribute;
                    `draw()` method then can be called to draw a plot;
                    for debugging purposes
        """
        self.kf_xyz = KalmanFilterLocalization()
        self.kf = KalmanFilterLocalization()
        if remember_history:
            self.history = {
                    'xyz from gps': [], # kartezske souradnice z GPS
                    'xyz': [], # kartezske souradnice vypocitane Kalmanovym filtrem
                    'pose3d': [], # navratove hodnoty
                }
        else:
            self.history = None
        # posledni poloha spocitana Kalmanovym filtrem
        self.last_xyz = None
        # cas posledni polohy spocitane Kalmanovym filtrem
        self.last_time = None
        # posledni poloha ziskana z odometrie a IMU
        self.last_xyz_from_imu = [0.0, 0.0, 0.0]
        # posledni orientace ziskana z IMU jako kvaternion
        self.last_orientation = None

    def update_xyz_from_gps(self, time, xyz_from_gps, gps_err = None):
        """
            Zpracuje a zapamatuje si dalsi souradnice ziskane z GPS.

            Args:
                xyz_from_gps (list of float): poloha [x, y, z] v metrech
                    odvozena z gps jako projekce do kartezske soustavy souradnic,
                    kde dve osy jsou tecne k povrchu Zeme a jedna osa je normalni k
                    povrchu Zeme
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`
                gps_err (list of float): chyby jednotlivych souradnic (smerodatne odchylky)
        """
        #print('update_xyz_from_gps:', time, xyz_from_gps, gps_err)
        self.kf.input(xyz_from_gps, time.total_seconds(), gps_err)
        time_in_seconds, xyz = self.kf.get_last_xyz()
        self.last_time = time
        self.last_xyz = xyz
        if self.history != None:
            self.history['xyz from gps'].append((time, xyz_from_gps))
            self.history['xyz'].append((time, xyz))

    def update_orientation(self, time, orientation):
        """
            Zpracuje a zapamatuje si dalsi orientaci robota v poradi.

            Args:
                orientation (list of float): kvaternion
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`
        """
        self.last_orientation = orientation
        
    def update_distance(self, time, data):
        """
            Zpracuje a zapamatuje si dalsi ujetou vzdalenost robota v poradi.

            Args:
                distance(float): ujeta vzdalenost; muze byt zaporna, pokud
                    robot couva
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`

            Returns (list): seznam o dvou polozkach
                1. seznam tri cisel ... souradnice v metrech
                2. seznam ctyr cisel ... kvaternion
        """
        # compute xyz from odometry and IMU
        # TODO if self.last_orientation == None: ...
        distance = data[0]
        distance_3d = quaternion.rotate_vector([distance, 0.0, 0.0], self.last_orientation)
        xyz_from_imu = [a + b for a, b in zip(self.last_xyz_from_imu, distance_3d)]
        self.last_xyz_from_imu = xyz_from_imu
        # compute angle between xyz from IMU and xyz from GPS
        # and rotate xyz from IMU
        xyz_from_imu_rotated = self.kf.rotate_xyz_from_imu(xyz_from_imu, time.total_seconds())
        # insert xyz from odometry and IMU into Kalman filter
        # (which works primarily with xyz form GPS)
        self.kf.input_imu(xyz_from_imu_rotated, time.total_seconds(), 10)
        time_in_seconds, xyz = self.kf.get_last_xyz()
        self.last_time = time
        self.last_xyz = xyz

    def get_pose3d(self, time = None):
        """
            Vrati extrapolovanou nebo posledni hodnotu pose3d.

            Args:
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`;

            Returns (list): seznam o dvou polozkach

                1. seznam tri cisel ... souradnice v metrech vyjadrujici polohu
                    robota;
                    pokud je zadana hodnota parametru `time`, tak vrati
                    extrapolovanou hodnotu pozice v tomto case;
                    pokud `time==None`, potom vrati posledni vypoctenou hodnotu
                    pozice
                2. seznam ctyr cisel ... kvaternion vyjadrujici orientaci robota;
                    pokud neni zjistitelny, vrati se `None`;
        """
        # TODO kvaternion neni extrapolovany, ale vraci se posledni hodnota
        # ziskana z IMU
        if time == None:
            result = [ self.last_xyz, self.last_orientation ]
        else:
            result = [ self.kf.get_xyz_estimate(time.total_seconds()), self.last_orientation ]
        if self.history != None:
            self.history['pose3d'].append( (time, result[0]) )
        return result

    def draw(self):
        """
            Draw a plot.
        """
        if self.history != None:
            import matplotlib.pyplot as plt
            drawn = ['xyz from gps', 'xyz', 'pose3d']
            colors = ['r.', 'g+-', 'bx', 'c+', 'mx', 'y.']
            for i in range(len(drawn)):
                x = []
                y = []
                for time, xyz in self.history[drawn[i]]:
                    x.append(xyz[0])
                    y.append(xyz[1])
                plt.plot(x, y, colors[i], label = drawn[i])
                plt.legend()
            plt.show()
