#!/usr/bin/env python3
"""
    Module launching the lego-car
"""
from libs.Robots import LegoCar
from libs.TrajectoryStuff_02 import Point, StraightLine, CircleLine, ClothoidLine, Pose


if __name__ == '__main__':

    # create car instance
    car = LegoCar()

    # trajectory tracking
    # move with desired velocities
    #car.velocity_move(vel_linear= 0.2, vel_angular=0.6, time=5.0)
    #t = CircleLine(Point(0, -1), Point(0, 0), Point(1, -1), "ccw", v=0.2)
    #t = AcademicClothoidLine(Point(2, 2), 0.2)
    t1 = ClothoidLine(Pose(0, 0, 0), 1, 0.349902, 3.793262, 0.2, type="in")
    t2 = ClothoidLine(Pose(2, 2, 2.5173395), 1, -0.349902, 3.793262, 0.2, type="out")
    trajectory = [t1, t2]
    #trajectory = [t]


    for t in trajectory:
        car.trajectory_move(trajectory=t)