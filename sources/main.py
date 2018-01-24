#!/usr/bin/env python3
"""
    Module launching the lego-car
"""
from libs.Robots import LegoCar
from libs.TrajectoryStuff import Point, StraightLine, CircleLine, ClothoidLine


if __name__ == '__main__':

    # create car instance
    car = LegoCar()

    # trajectory tracking
    # move with desired velocities
    #car.velocity_move(vel_linear= 0.2, vel_angular=0.2, time=3.0)
    #t = CircleLine(Point(0, -1), Point(0, 0), Point(1, -1), "ccw", v=0.2)
    t = ClothoidLine(Point(2, 2), v=0.2)
    car.trajectory_move(t)
    #t2 = CircleLine(Point(-2, 1), Point(-1, 1), Point(-2, 0), 0.35)
    #trajectory = [t1, t2]

    #for t in trajectory:
        #car.trajectory_move(trajectory=t)