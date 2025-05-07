from abc import ABC, abstractmethod

class Tracker(ABC):
    @abstractmethod
    def input_gps_xyz(self, time, xyz):
        pass

    @abstractmethod
    def input_distance_travelled(self, time, distance):
        pass

    @abstractmethod
    def input_orientation(self, time, data):
        pass

    @abstractmethod
    def get_pose3d(self):
        pass
