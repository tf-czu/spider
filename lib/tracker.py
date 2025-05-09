from abc import ABC, abstractmethod

class Tracker(ABC):
    """
        This is an abstract class that describes the interface of the derived
            classes which are designed to compute trajectory from asynchronously
            provided GPS, odometry, and IMU data utilizing Kalman filter.

        Derived classes:
            
            * TrackerLeastSquares
            * TrackerKalman

        Input: two sources of 3D position, asynchronously provided:

            * 3D cartesian coordinates [in meters] computed from GPS:
                This value may be affected by a significant non-Gaussian error (the
                correct position can differ by several meters), however, this error
                is not affected by the distance travelled.
                Hence, the relative error decreases with when the distance
                travelled increases and the position is well reliable on long
                distances.
                See: Tracker.input_gps_xyz()

            * 3D cartesian coordinates [in meters] computed from odometry and IMU:
                Odometry provides relative changes of the distance travelled [in
                meters] while IMU provides the orientation of the robot as a
                quaternion a+bi+cj+dk represented by the list `[b,c,d,a]`.
                This is in accord with the [ROS standard]
                (http://wiki.ros.org/tf2/Tutorials/Quaternions), see also
                osgar/lib/quaternion.py.
                Both odometry and IMU are affected by errors which are more or less
                constant in time, however, the longer distance the robot travels,
                the grater impact these errors have.
                See: Tracker.input_distance_travelled(),
                    Tracker.input_orientation()

        Output: position given by 3D coordinates and orientation given by a quaternion

            See: Tracker.get_pose3d()

        Author:

            * Milan Petrík (petrikm@tf.czu.cz)
    """

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

