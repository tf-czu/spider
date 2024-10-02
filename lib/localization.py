import numpy as np
import osgar.lib.quaternion as quaternion
from lib.kalman_localization import KalmanFilterLocalization

class Localization:
    def __init__(self):
        self.kf_xyz = KalmanFilterLocalization()
        self.kf = KalmanFilterLocalization()
        self.history = {
                # seznam tri cisel; poloha vypocitana Kalmanovym filtrem z GPS
                # a IMU
                'xyz': [],
                # seznam ctyr cisel; orientace jako kvaternion; zatim jenom z
                # IMU, v budoucnu se to mozna bude pocitat
                'orientation': [],
                # [xyz, orientation]
                'pose3d': [],
                'distance': [],
                'gps planar': [],
                'xyz from gps': [],
                'xyz from imu': [],
                'xyz from imu rotated': [],
                'gps planar filtered': [],
                'xyz from imu filtered': [],
                'angle': [],
                'scale': [],
                'returned': [],
            }
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
                gps_err (list of float): chyby jednotlivych souradnic
        """
        self.kf.input(xyz_from_gps, time.total_seconds(), 10)
        time_in_seconds, xyz = self.kf.get_last_xyz()
        self.last_time = time
        self.last_xyz = xyz
        # history
        self.history['xyz from gps'].append( (time, xyz_from_gps) )
        self.history['xyz'].append((time, xyz))
        self.history['pose3d'].append((time, self.get_pose3d(time)[0]))

    def update_orientation(self, time, orientation):
        """
            Zpracuje a zapamatuje si dalsi orientaci robota v poradi.

            Args:
                orientation (list of float): kvaternion
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`
        """
        self.last_orientation = orientation
        # history
        self.history['orientation'].append((time, orientation))
        self.history['pose3d'].append((time, self.get_pose3d(time)[0]))
        
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
        # compute angle between xyz from imu and xyz from GPS
        # and rotate xyz from imu
        xyz_from_imu_rotated = self.kf.rotate_xyz_from_imu(xyz_from_imu, time.total_seconds())
        # insert xyz from odometry and IMU into Kalman filter
        # (which works primarily with xyz form GPS)
        self.kf.input_imu(xyz_from_imu_rotated, time.total_seconds(), 10)
        time_in_seconds, xyz = self.kf.get_last_xyz()
        self.last_time = time
        self.last_xyz = xyz
        # history
        self.history['xyz'].append((time, xyz))
        self.history['xyz from imu'].append((time, xyz_from_imu))
        if not xyz_from_imu_rotated is None:
            self.history['xyz from imu rotated'].append((time, list(xyz_from_imu_rotated)))
        if self.kf.angle_filter.get() != None:
            self.history['angle'].append((time, self.kf.angle_filter.get()))
        if self.kf.scale_filter.get() != None:
            self.history['scale'].append((time, self.kf.scale_filter.get()))
        self.history['pose3d'].append((time, self.get_pose3d(time)[0]))
        #return self.get_pose3d(time) # TODO vracet nebo nevracet?

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
            return [ self.last_xyz, self.last_orientation ]
        else:
            return [ self.kf.get_xyz_estimate(time.total_seconds()), self.last_orientation ]

    def draw(self):
        """
            Draw a plot with the --draw parameter.
        """
        import matplotlib.pyplot as plt
        drawn = ['xyz from gps', 'xyz from imu', 'xyz from imu rotated', 'xyz', 'pose3d']
        colors = ['r.', 'g.', 'b.', 'c+', 'mx', 'y.']
        for i in range(len(drawn)):
            x = []
            y = []
            for time, xyz in self.history[drawn[i]]:
                x.append(xyz[0])
                y.append(xyz[1])
            plt.plot(x, y, colors[i], label = drawn[i])
            plt.legend()
        plt.show()
        ## draw 'gps planar filtered'
        #draw_gps_planar_f_x = []
        #draw_gps_planar_f_y = []
        #for time, gps_planar_f in self.history['gps planar filtered']:
        #    draw_gps_planar_f_x.append(gps_planar_f[0])
        #    draw_gps_planar_f_y.append(gps_planar_f[1])
        #plt.plot(draw_gps_planar_f_x, draw_gps_planar_f_y, 'r.', label = 'gps planar filtered')
        ## draw 'xyz from imu'
        #draw_xyz_x = []
        #draw_xyz_y = []
        #draw_xyz_z = []
        #for time, xyz in self.history['xyz from imu']:
        #    draw_xyz_x.append(xyz[0])
        #    draw_xyz_y.append(xyz[1])
        #    draw_xyz_z.append(xyz[2])
        #plt.plot(draw_xyz_x, draw_xyz_y, 'g.', label = 'xyz from imu')
        ## draw 'xyz from imu filtered'
        ##draw_xy_f_x = []
        ##draw_xy_f_y = []
        ##for time, xy_f in self.history['xyz from imu filtered']:
        ##    draw_xy_f_x.append(xy_f[0])
        ##    draw_xy_f_y.append(xy_f[1])
        ##plt.plot(draw_xy_f_x, draw_xy_f_y, 'b.', label = 'xyz from imu filtered')
        ## draw 'xyz from imu rotated'
        #draw_xy_f_x = []
        #draw_xy_f_y = []
        #for time, xy_f in self.history['xyz from imu rotated']:
        #    draw_xy_f_x.append(xy_f[0])
        #    draw_xy_f_y.append(xy_f[1])
        #plt.plot(draw_xy_f_x, draw_xy_f_y, 'c.', label = 'xyz from imu rotated')
        #plt.legend()
        #plt.figure()
        #t_a = []
        #ang = []
        #for (t, a) in self.history['angle']:
        #    t_a.append(t.total_seconds())
        #    ang.append(a * 180 / np.pi)
        #plt.plot(t_a, ang, 'b.', label = 'angle')
        #plt.legend()
        #plt.figure()
        #t_s = []
        #sca = []
        #for (t, s) in self.history['scale']:
        #    t_s.append(t.total_seconds())
        #    sca.append(s)
        #plt.plot(t_s, sca, 'g.', label = 'scale')
        #plt.legend()
        #plt.show()
