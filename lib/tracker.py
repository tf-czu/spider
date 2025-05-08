from abc import ABC, abstractmethod

class Tracker(ABC):
    @abstractmethod
    def input_gps_xyz(self, time, xyz):
        """
            Process next position obtained from GPS.

            Args:
                xyz (list of float): list of three values `[x, y, z]`
                    representing cartesian coordinates derived from GPS data
        """
        pass

    @abstractmethod
    def input_distance_travelled(self, time, distance):
        """
            Process next (relative) distance travelled obtained from odometry.

            Args:
                distance (float): distance travelled in meters; relative from
                    the last measurement
        """
        pass

    @abstractmethod
    def input_orientation(self, time, orientation):
        """
            Process next orientation quaternion obtained from IMU.

            Args:
                data (list of float): orientation of the robot as a quaternion
                    a+bi+cj+dk given by the list `[b,c,d,a]`
        """
        pass

    @abstractmethod
    def get_pose3d(self):
        """
            Returns (list): `[xyz, ori]` where:

                * xyz ... 3D position given by the list `[x,y,z]`
                * ori ... orientation as a quaternion a+bi+cj+dk given by the
                    list `[b,c,d,a]`
        """
        pass

    @abstractmethod
    def get_odo_xyz(self):
        """
            Returns (list): `[x,y,z]` coordinates derived from odometry and IMU
        """
        pass

