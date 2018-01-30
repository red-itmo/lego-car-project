from math import pi, copysign
import numpy as np
import random
from Auxilary import *
from TrajectoryStuff_02 import Pose

k_max = 1.5
k_max = 0.9 * k_max
L = 0.165
v_min = 0.05
Omega_max = 10
sigma_max = Omega_max / L
alpha_max = sigma_max / v_min


class EESPlaner:
    def getTrajectory(self, start_pose, goal_pose, v_des, reverse=False):
        if reverse:
            init_pose = goal_pose
            goal_pose = start_pose
        else:
            init_pose = start_pose

        pose = np.zeros(shape=(3,3))
        pose[0:] = transformCoords(goal_pose, np.array([[init_pose.point.x, init_pose.point.y, init_pose.angle]]), 'b')

        if pose[0,2] > pi:
            pose[0,2] = pose[0,2] - 2 * pi
        elif pose[0,2] < -pi:
            pose[0,2] = pose[0,2] + 2 * pi

        x1 = pose[0,0]
        y1 = pose[0,1]
        theta_1 = pose[0,2]

        delta = np.zeros(shape=(2, 1))
        k = np.zeros(shape=(2, 1))

        alphas_are_not_well = True
        while alphas_are_not_well:
            if 0 <= theta_1 and theta_1 < pi:
                flag = False
                while flag != True:
                    flag = True
                    delta[0,0] = random.uniform(-pi / 2, 0.5 * (pi - theta_1))
                    if delta[0,0] == 0 or delta[0,0] == -theta_1 + pi:
                        flag = False
            elif -pi <= theta_1 and theta_1 < 0:
                flag = False
                while flag != True:
                    flag = True
                    delta[0,0] = random.uniform(-0.5 * (pi - theta_1), pi / 2)
                    if delta[0,0] == 0 or delta[0,0] == -theta_1 -pi:
                        flag = False

            flag = False
            while flag != True:
                flag = True
                k[0,0] = random.uniform(-k_max, k_max)
                if k[0,0] == 0 or abs(y1 + D([2 * delta[0,0]], theta_1) / k[0,0]) < B(2 * delta[0,0] + theta_1) / k_max:
                    flag = False

            delta[1,0] = -delta[0,0] - 0.5 * theta_1

            k[1,0] = B(-2 * delta[1,0]) / (D([2 * delta[0,0]], theta_1) / k[0,0] - x1)

            pose[2,0] = -(A(-2 * delta[1,0]) / k[1,0] - C([2 * delta[0,0]], theta_1) / k[0,0] - x1)
            gamma = np.ndarray(shape=(3, 1))
            gamma[2,:] = -copysign(1, pose[2,0])
            s_end = np.ndarray(shape=(3, 1))
            s_end[2,:] = abs(pose[2, 0])
            pose[2, 1] = 0

            alpha = np.ndarray(shape=(2, 1))
            v = np.ndarray(shape=(2, 1))

            for i in range(0, 2):
                gamma[i,0] = copysign(1, delta[i,0]) * copysign(1, k[i,0])
                s_end[i,0] = abs(2 * delta[i] / k[i])
                alpha[i,0] = gamma[i] * copysign(1, k[i]) * abs(k[i] / s_end[i])
                if sigma_max / abs(alpha[i]) < v_des:
                    v[i,0] = sigma_max / abs(alpha[i])
                else:
                    v[i,:] = v_des

            alphas_are_not_well = False
            for i in range(0, 2):
                if alpha[i,0] > alpha_max:
                    alphas_are_not_well = True

        length = 2 * sum(s_end) - s_end[2,0]

        if reverse:
            descr = []
            descr.append(["StraightLine", Pose(0, 0, 0), Pose(pose[2,0], pose[2, 1], pose[2, 2]), s_end[2,0], v_des])
            for i in range(0, 2):
                descr.append(["ClothoidLine", -gamma[i], alpha[i], s_end[i], 'in', v[i]])
                descr.append(["ClothoidLine", -gamma[i], -alpha[i], s_end[i], 'out', v[i]])
        else:
            descr = []
            for i in range(0, 2):
                descr.append(["ClothoidLine", gamma[i], alpha[i], s_end[i], 'in', v[i]])
                descr.append(["ClothoidLine", gamma[i], -alpha[i], s_end[i], 'out', v[i]])
            descr.append(["StraightLine", Pose(0, 0, 0), Pose(pose[2,0], pose[2, 1], pose[2, 2]), s_end[2, 0], v_des])

        aux_pose = Pose(pose[0,0], pose[0, 1], pose[0,2])
        robot_poses = []
        for i in range(0,5):
            poses = calcPathElementPoints(descr[i], aux_pose, 0.05)
            aux_pose = Pose(poses[-1,0], pose[-1, 1], pose[-1, 2])
            add_poses = transformCoords(goal_pose, poses)
            robot_poses.append(add_poses)

        return descr, length, robot_poses