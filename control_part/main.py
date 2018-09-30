#!/usr/bin/env python3
"""
    Module launching the lego-car
"""
from libs.Robots import LegoCar
from libs.TrajectoryStuff import Point, StraightLine, CircleLine, ClothoidLine, Pose
import libs.Client as Client
from libs.TrajectoryParser import TrajectoryParser
import libs.Localization as Loc
from math import cos, sin, pi
from numpy import array, asscalar

if __name__ == '__main__':

    # open a socket to get data from computer
    sock = Client.Client(3000)

    # get data socket
    while True:
        data = sock.get_data()
        if data:
            break

    # parse received data
    init_pose = data[0]
    print("INIT POSE: ", init_pose)
    path = data[1]

    # creating instance of a car
    car = LegoCar(init_pose, sock)

    # creating object for parsing receives path
    trajectoryHandler = TrajectoryParser(init_pose)

    # decoding the path
    trajectory = trajectoryHandler.decode_trajectory(path)
    for t in trajectory:
        print("New trajectory")
        car.trajectory_move(trajectory=t)
