#!/usr/bin/env python3
"""
    Module launching the parking lego-car
"""
from libs.Robots import LegoCar
from libs.TrajectoryStuff import Point, StraightLine, ParkingLine
from libs.Mapping import Mapping


if __name__ == '__main__':

    # create car instance
    car = LegoCar()

    # init mapping
    susf, susr = car.getDevice('US_SENSORS')
    mapping = Mapping(car, susf, susr)

    # simultaneous forward movement and mapping
    trajectory_line = StraightLine(Point(0, 0), Point(4, 0), 0.2)
    car.move(trajectory=trajectory_line, mapping=mapping)   # mapping mode

    # finding points x_l and x_r ont the map
    mapping.filterMap()
    x_l, x_r = mapping.findParkingPlace()

    # parking
    x, y, __, __, __, __ = car.getState()
    parking_line = ParkingLine(Point(x, y), x_l, x_r, 1, car.D / 2, 0.03, 0.03, 0.3)
