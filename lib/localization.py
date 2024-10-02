
import numpy as np

from osgar.lib.quaternion import rotate_vector

from lib.kalman_9 import Acc3DKalmanFilter

def interpolate(x, x_a, y_a, x_b, y_b):
    """
        Vrati afinne interpolovanou hodnotu mezi body [x_a, y_a] a [x_b, y_b]
            na zaklade parametru `x`.
    """
    if abs(x_b - x_a) < 0.000001:
        return (y_a + y_b) / 2
    else:
        return y_a + ( (x - x_a) * (y_b - y_a) / (x_b - x_a) )

class Localization:
    def __init__(self):
        self.kf_xyz = Acc3DKalmanFilter()
        self.kf_gps = Acc3DKalmanFilter()
        self.history = {
                'distance': [],
                'orientation': [],
                'gps planar': [],
                'position from IMU': [],
                'position from IMU rotated': [],
                'gps planar filtered': [],
                'position from IMU filtered': [],
                'pose3d': [],
                'angle': [],
                'scale': [],
            }

    def update_gps_planar(self, time, gps_planar):
        """
            Zpracuje a zapamatuje si dalsi GPS souradnice v poradi.

            Args:
                gps_planar (list of float): poloha [x, y] v metrech odvozena z gps
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`
        """
        self.history['gps planar'].append( (time, gps_planar) )
        # Kalman filter
        time_in_seconds = time.total_seconds()
        self.kf_gps.input_measurement(gps_planar + [0], time_in_seconds, 10)
        gps_planar_filtered = self.kf_gps.get_position_estimate(time_in_seconds)[:2]
        self.history['gps planar filtered'].append( (time, gps_planar_filtered) )

    def update_orientation(self, time, orientation):
        """
            Zpracuje a zapamatuje si dalsi orientaci robota v poradi.

            Args:
                orientation (list of float): kvaternion
                time (datetime.timedelta): (absolutni) cas;
                    cas ve vterinach lze ziskat pomoci `time.total_seconds()`
        """
        self.history['orientation'].append( (time, orientation) )
        
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
        # compute xyz position from odometry and IMU
        distance = data[0]
        self.history['distance'].append( (time, distance) )
        orientation_time, orientation = self.history['orientation'][-1]
        distance_3D = [distance * o for o in orientation]
        distance_3D = rotate_vector([distance, 0.0, 0.0], orientation)
        if len(self.history['position from IMU']) == 0:
            last_xyz = [0.0, 0.0, 0.0]
        else:
            last_xyz_time, last_xyz = self.history['position from IMU'][-1]
        xyz = [a + b for a, b in zip(last_xyz, distance_3D)]
        self.history['position from IMU'].append((time, xyz))
        # compare xyz position with the position given by GPS
        xyz_rotated = self.kf_gps.rotate_IMU_position(xyz, time.total_seconds())
        #print('xyz_rotated', xyz_rotated, type(xyz_rotated))
        if not xyz_rotated is None:
            self.history['position from IMU rotated'].append((time, list(xyz_rotated)))
        if self.kf_gps.angle_filter.get() != None:
            self.history['angle'].append((time, self.kf_gps.angle_filter.get()))
        if self.kf_gps.scale_filter.get() != None:
            self.history['scale'].append((time, self.kf_gps.scale_filter.get()))
        # IMU Kalman filter
        time_in_seconds = time.total_seconds()
        self.kf_xyz.input_measurement(xyz, time_in_seconds, 10)
        xy_filtered = self.kf_xyz.get_position_estimate(time_in_seconds)[:2]
        self.history['position from IMU filtered'].append( (time, xy_filtered) )
        # GPS Kalman filter
        self.kf_gps.input_IMU_measurement(xyz, time_in_seconds, 10)
        # output
        pose3d = [xyz, orientation]
        self.history['pose3d'].append( (time, pose3d) )
        return pose3d

    def get_pose3d(self):
        """
            Vrati posledni hodnotu pose3d.

            Returns (list): seznam o dvou polozkach
                1. seznam tri cisel ... souradnice v metrech vyjadrujici polohu robota
                2. seznam ctyr cisel ... kvaternion vyjadrujici orientaci robota
        """
        time, pose3d = self.history['pose3d'][-1]
        return pose3d

    def draw(self):
        """
            Draw a plot with the --draw parameter.
        """
        import matplotlib.pyplot as plt
        # draw 'gps planar'
        draw_gps_planar_x = []
        draw_gps_planar_y = []
        for time, gps_planar in self.history['gps planar']:
            draw_gps_planar_x.append(gps_planar[0])
            draw_gps_planar_y.append(gps_planar[1])
        plt.plot(draw_gps_planar_x, draw_gps_planar_y, 'm.', label = 'gps planar')
        # draw 'gps planar filtered'
        draw_gps_planar_f_x = []
        draw_gps_planar_f_y = []
        for time, gps_planar_f in self.history['gps planar filtered']:
            draw_gps_planar_f_x.append(gps_planar_f[0])
            draw_gps_planar_f_y.append(gps_planar_f[1])
        plt.plot(draw_gps_planar_f_x, draw_gps_planar_f_y, 'r.', label = 'gps planar filtered')
        # draw 'position from IMU'
        draw_xyz_x = []
        draw_xyz_y = []
        draw_xyz_z = []
        for time, xyz in self.history['position from IMU']:
            draw_xyz_x.append(xyz[0])
            draw_xyz_y.append(xyz[1])
            draw_xyz_z.append(xyz[2])
        plt.plot(draw_xyz_x, draw_xyz_y, 'g.', label = 'position from IMU')
        # draw 'position from IMU filtered'
        draw_xy_f_x = []
        draw_xy_f_y = []
        for time, xy_f in self.history['position from IMU filtered']:
            draw_xy_f_x.append(xy_f[0])
            draw_xy_f_y.append(xy_f[1])
        plt.plot(draw_xy_f_x, draw_xy_f_y, 'b.', label = 'position from IMU filtered')
        # draw 'position from IMU rotated'
        draw_xy_f_x = []
        draw_xy_f_y = []
        for time, xy_f in self.history['position from IMU rotated']:
            draw_xy_f_x.append(xy_f[0])
            draw_xy_f_y.append(xy_f[1])
        plt.plot(draw_xy_f_x, draw_xy_f_y, 'c.', label = 'position from IMU rotated')
        plt.legend()
        plt.figure()
        t_a = []
        ang = []
        for (t, a) in self.history['angle']:
            t_a.append(t.total_seconds())
            ang.append(a * 180 / np.pi)
        plt.plot(t_a, ang, 'b.', label = 'angle')
        plt.legend()
        plt.figure()
        t_s = []
        sca = []
        for (t, s) in self.history['scale']:
            t_s.append(t.total_seconds())
            sca.append(s)
        plt.plot(t_s, sca, 'g.', label = 'scale')
        plt.legend()
        plt.show()
