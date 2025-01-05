import numpy as np
import math
import datetime
import time

class AngleScaleEstimator:
    """
        Takes position (and time?) data from two sources (gps and IMU)
        and estimates rotation and scale between them

        Input:
            * `update()` ... processes a gps input and imu input 

        Output:
            * `get_angle()`
            * `get_scale()`
    """

    def __init__(self, history_length = math.inf):
        """
            history_length (int): from how many items in the history the
                resulting angle and scale is computed;
                `math.inf` (default) means that the resulting angle and scale
                are computed from all the inputs in the history
        """
        self.reset()
        # measurements closer to origin than `minimal_distance_to_accept` are
        # not processed at all, maximum weight gets measurements at least
        # `distance_with_full_weight` from the origin 
        self.minimal_distance_to_accept = 10 
        self.distance_with_full_weight = 100
        # the angle and scale can be calculated with respect to any point, i.e.
        # self.origin
        self.origin = [0.0,0.0]
        # the averaging can be computed over all input data or over last
        # `history_length` of them, in that case the history is stored in
        # `history`
        self.finite_history = (history_length != math.inf) # to distinguish between the two types of the estimator
        if history_length < math.inf:
            self.history_length = history_length
        
    def reset(self):
        """
            Force initial settings.
        """
        self.angle = None
        self.scale = None
        self.total_weight = 0    # used when `history_length` is infinite
        self.angle_history = []  # used when `history_length` is finite
        self.scale_history = []  # used when `history_length` is finite
        self.weight_history = [] # used when `history_length` is finite

    def set_origin(self, origin):
        self.origin = origin
        self.reset()

    def get_distance_from_origin(self, position):
        """
            Returns (float): the Euclidean distance of `position` from
            self.origin 
        """
        # assuming the starting position to be [0, 0]
        #return np.sqrt(position[0]**2+position[1]**2)
        return np.sqrt((self.origin[0]-position[0])**2+(self.origin[1]-position[1])**2)

    def weight_by_distance(self, d):
        """
            Close to the origin the weight is zero, then it linearly increases
                up to 1, if the distance from the origin is greater than
                self.distance_with_full_weight it gets 1.
        """
        if d < self.minimal_distance_to_accept:
            return 0
        if self.minimal_distance_to_accept <= d <= self.distance_with_full_weight:
            return max(0.1, (d-self.minimal_distance_to_accept)/(self.distance_with_full_weight-self.minimal_distance_to_accept))
        return 1

    def calculate_angle(self, b1, b2):
            """
                Args:
                    b1 (list of float): coordinates [x, y]
                    b2 (list of float): coordinates [x, y]

                Returns (float): float values in in (-pi, pi>; angle at [0, 0]
                    under which are the points seen, 

                Warning: this function is not continuous, in the case of angles
                    close to pi, it may not work properly
            """
            #recalculating with respect to self.origin
            for i in range(2):
                b1[i] -= self.origin[i] 
                b2[i] -= self.origin[i] 


            c1 = np.complex128(b1[0]+b1[1]*1j)
            c2 = np.complex128(b2[0]+b2[1]*1j)
            return np.angle(c1/c2)

    def calculate_scale(self, b1, b2):
            """
                Args:
                    b1: souradnice x, y bodu v rovine
                    b2: souradnice x, y bodu v rovine

                Returns (float): the ratio of distances of the points from the origin
            """
            return self.get_distance_from_origin(b1)/self.get_distance_from_origin(b2)

    def update(self, gps, imu):
        """
            Updates the estimates of the angle and scale based on a new data,
            assumes that the two data are from the same time

            Args:
                gps: list of floats, uses only the first two coordinates,
                can have three

                imu: list of floats, uses only the first two coordinates,
                can have three
        """
        gps = gps[:2]
        imu = imu[:2]
        # weight of the current measurement is given by the closer to origin
        # point of the two
        gps_distance = self.get_distance_from_origin(gps)
        imu_distance = self.get_distance_from_origin(imu)
        weight = min(self.weight_by_distance(gps_distance), self.weight_by_distance(imu_distance))
        if weight != 0:
            scale = self.calculate_scale(gps, imu)
            angle = self.calculate_angle(gps, imu)
            if self.finite_history:
                if len(self.angle_history) == self.history_length:
                    # history is full and the last element must be deleted and
                    # subtracted
                    # in this case self.angle and self.scale should not be None
                    # by definition
                    angle_to_remove = self.angle_history.pop()
                    scale_to_remove = self.scale_history.pop()
                    weight_to_remove = self.weight_history.pop()
                    # removing old scale
                    self.scale = (self.total_weight*self.scale - weight_to_remove*scale_to_remove)/(self.total_weight - weight_to_remove)
                    # removing old angle
                    self_vector = np.array([math.cos(self.angle), math.sin(self.angle)])
                    v1 = self_vector*self.total_weight
                    vector_to_remove = np.array([math.cos(angle_to_remove), math.sin(angle_to_remove)])
                    v2 = vector_to_remove*weight_to_remove
                    v = v1-v2
                    self.angle = self.calculate_angle(v,[1,0])
                    # removing old weight
                    self.total_weight -= weight_to_remove
                # adding new measurement
                # case when only adding measurement to the history, it is
                # the same as in infinite history case, only the history is
                # being created
                self.angle_history.insert(0, angle)
                self.scale_history.insert(0, scale)
                self.weight_history.insert(0, weight)
                if self.scale is not None:
                    self.scale = (self.total_weight*self.scale + weight*scale)/(self.total_weight+weight)
                else:
                    self.scale = scale
                if self.angle is not None:
                    self_vector = np.array([math.cos(self.angle), math.sin(self.angle)])
                    v1 = self_vector*self.total_weight
                    new_vector = np.array([math.cos(angle), math.sin(angle)])
                    v2 = new_vector*weight
                    v = v1+v2
                    self.angle = self.calculate_angle(v,[1,0])
                else:
                    self.angle = angle
                self.total_weight += weight
            else:
                # The case with infinite history
                if self.scale is not None:
                    self.scale = (self.total_weight*self.scale + weight*scale)/(self.total_weight+weight)
                else:
                    self.scale = scale

                if self.angle is not None:
                   self_vector = np.array([math.cos(self.angle), math.sin(self.angle)])
                   v1 = self_vector*self.total_weight
                   new_vector = np.array([math.cos(angle), math.sin(angle)])
                   v2 = new_vector*weight
                   v = v1+v2
                   self.angle = self.calculate_angle(v,[1,0])
                else:
                    self.angle = angle

                self.total_weight += weight

    def get_angle(self):
        """
            Returns the estimate of the angle, can be None
        """
        return self.angle

    def get_scale(self):
        """
            Returns the estimate of the scale, can be None
        """
        return self.scale

