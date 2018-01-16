#!/usr/bin/env python3
"""
    Module launching the lego-car
"""
from libs.Robots import LegoCar
from libs.TrajectoryStuff import Point, StraightLine


if __name__ == '__main__':

    # create car instance
    car = LegoCar()

    # trajectory tracking
    trajectory_line = StraightLine(Point(0, 0.5), Point(4, 0.5), 0.2)
    car.move(trajectory=trajectory_line)
