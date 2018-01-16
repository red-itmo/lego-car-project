# coding: utf-8
"""
    Class contains
"""
from math import cos, sin, hypot

from libs.utils import matmult, meanFilter
from libs.TrajectoryStuff import Point

MAX_VIEW_DISTANCE = 0.5     # [m] how far the robot can see
NEED_FREE_SPACE = 0.3       # [m] how


class Mapping:
    """
        Class contains methods to create and use <<map>>
        Map consist of set of Point instances.
    """

    def __init__(self, car, us_sensor_front, us_sensor_rear):
        self.car = car
        self.us_front = us_sensor_front
        self.us_rear = us_sensor_rear
        self.max_view_distanse = MAX_VIEW_DISTANCE
        self.mean = 0

        # map contains two set of measurements:
        # for front sensor and for rear sensor
        self.the_map = {
            'obstacles': {'front': [], 'rear': []}
        }

    def updateMap(self, x_current, y_current, theta):
        """ Add new points to map on the current time """
        H = [[cos(theta), -sin(theta), 0, x_current],
             [sin(theta), cos(theta), 0, y_current],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]

        # cm to meters
        dist_front = self.us_front.distance_centimeters * 0.01
        dist_rear = self.us_rear.distance_centimeters * 0.01

        if dist_front < self.max_view_distanse:
            # measured point in robot frame
            point_front = [self.car.US_FRONT[0],
                           -(dist_front + self.car.US_FRONT[1]),
                           0,
                           1]
            # measured point in base frame
            point_front_0 = matmult(H, point_front)
            self.the_map['obstacles']['front'].append(Point(point_front_0[0], point_front_0[1]))

        if dist_rear < self.max_view_distanse:
            # measured point in robot frame
            point_rear = [self.car.US_REAR[0],
                          -(dist_rear + self.car.US_REAR[1]),
                          0,
                          1]
            # measured point in base frame
            point_rear_0 = matmult(H, point_rear)
            self.the_map['obstacles']['rear'].append(Point(point_rear_0[0], point_rear_0[1]))

    def filterMap(self):
        """Remove all points further that mean value of all points"""
        print("Quantity points with noize: ".format(self.the_map['obstacles']['front'] + self.the_map['obstacles']['rear']))
        self.the_map['obstacles']['front'], self.mean = meanFilter(self.the_map['obstacles']['front'])
        self.the_map['obstacles']['rear'], self.mean = meanFilter(self.the_map['obstacles']['rear'])
        print("Quantity points without noize: ".format(self.the_map['obstacles']['front'] + self.the_map['obstacles']['rear']))

    def findParkingPlace(self):
        """
            The search space where it is possible to park the car.
            !!! Using only front sensor.
            :returns list of points of start and end free space [x_l, x_r]
        """
        parking_place = []
        last_point = self.the_map['obstacles']['front'][0]
        for point in self.the_map['obstacles']['front'][1:]:
            if abs(point.x - last_point.x) > NEED_FREE_SPACE:
                parking_place.append(last_point)
                parking_place.append(point)
                break
            last_point = point
        return parking_place
