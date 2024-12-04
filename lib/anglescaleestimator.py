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

    def __init__(self):
        self.reset()

        # maximal weight of a new measurement

        # measurements closer to origin than self.minimal_distance_to_accept are not
        # processed at all, maximum weight gets measurements at least 
        # self.distance_with_full_weight from the origin 
        self.minimal_distance_to_accept = 10 
        self.distance_with_full_weight = 50
        
    def reset(self):
        """
            Force initial settings.
        """
        self.angle = None
        self.scale = None
        self.number_of_measurements = 0

    def get_distance_from_origin(self, position):
        # assuming the starting position to be [0,0]
        return np.sqrt(position[0]**2+position[1]**2)

    def weight_by_distance(self, d):
        """
            Close to the origin the weight is zero, than it linearly increases
            up to 1, if the distance from the origin is greater
            than self.distance_with_full_weight it gets 1
        """
        if d < self.minimal_distance_to_accept:
            return 0
        if self.minimal_distance_to_accept <= d <= self.distance_with_full_weight:
            return (d-self.minimal_distance_to_accept)/(self.distance_with_full_weight-self.minimal_distance_to_accept)
        return 1

    def calculate_angle(self, b1, b2):
            """
                Args:
                    b1: souradnice x, y bodu v rovine
                    b2: souradnice x, y bodu v rovine
                Returns angle at [0,0] at which are the points seen, 
               lis in (-pi, pi>
                Warning: this function is not continuous, in the case of angles
                close to pi, it may not work properly
            """
            c1 = np.complex128(b1[0]+b1[1]*1j)
            c2 = np.complex128(b2[0]+b2[1]*1j)
            return np.angle(c1/c2)

    def calculate_scale(self, b1, b2):
            """
                Args:
                    b1: souradnice x, y bodu v rovine
                    b2: souradnice x, y bodu v rovine
                Returns the ratio of distances of the points from the origin
            """
            #print(b1)
            #print(b2)
            #print(self.get_distance_from_origin(b1)/self.get_distance_from_origin(b2))
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
        #print("weight", weight)
        if weight != 0:
            scale = self.calculate_scale(gps, imu)
            if self.scale is not None:
                #if abs(new_scale - self.scale) > 0.1:
                #   print("reseting scale too much")
                #   time.sleep(1)
                self.scale = (self.number_of_measurements*self.scale + weight*scale)/(self.number_of_measurements+weight)
            else:
                self.scale = scale

            angle = self.calculate_angle(gps, imu)
            if self.angle is not None:
               self_vector = np.array([math.cos(self.angle), math.sin(self.angle)])
               v1 = self_vector*self.number_of_measurements
               new_vector = np.array([math.cos(angle), math.sin(angle)])
               v2 = new_vector*weight
               v = v1+v2
               #new_angle = self.calculate_angle(v,[1,0])
               #if abs(new_angle - self.angle) > 0.1:
               #    print("reseting angle too much")
               #    time.sleep(1)
               self.angle = self.calculate_angle(v,[1,0])
            else:
                self.angle = angle

            self.number_of_measurements += 1


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

