from math import cos, sin, pi, copysign
import numpy as np
import random
from libs.Auxilary import *
import sys

k_max = 1.5
k_max = 0.9 * k_max
L = 0.165
v_min = 0.05
Omega_max = 10
sigma_max = Omega_max / L
alpha_max = sigma_max / v_min


class EESPlaner:
    def getTrajectory(self, start_pose, goal_pose, v_des, mode):
        if len(sys.argv) == 4 and mode == 'r':
            init_pose = goal_pose
            goal_pose = start_pose
        else:
            init_pose = start_pose

        pose = transformCoords(goal_pose, init_pose, 'b')

        if pose[0,2] > pi:
            pose[0,2] = pose[0,2] - 2 * pi
        elif pose[0,2] < -pi:
            pose[0,2] = pose[0,2] + 2 * pi

        x1 = pose[0,0]
        y1 = pose[0,1]
        theta_1 = pose[0,2]

        alphas_are_not_well = True
        while alphas_are_not_well:
            if 0 <= theta_1 and theta_1 < pi:
                flag = False
                while flag != True:
                    flag = True
                    delta = np.zeros(shape=(2,1))
                    delta[0,0] = random.randrange(-pi / 2, 0.5 * (pi - theta_1))
                    if delta[0,0] == 0 or delta[0,0] == -theta_1 + pi:
                        flag = False
            elif -pi <= theta_1 and theta_1 < 0:
                flag = False
                while flag != True:
                    flag = True
                    delta = np.zeros(shape=(2,1))
                    delta[0,0] = random.randrange(-0.5 * (pi - theta_1), pi / 2)
                    if delta[0,0] == 0 or delta[0,0] == -theta_1 -pi:
                        flag = False

        flag = False
        while flag != True:
            flag = True
            k = np.zeros(shape=(2,1))
            k[0,0] = random.randrange(-k_max, k_max)
            if k[0,0] == 0 or abs(y1 + D(2 * delta[0,0], theta_1) / k[0,0]) < B(2 * delta[0,0] + theta_1) / k_max:
                flag = False

        delta[0,1] = -delta[0,0] - 0.5 * theta_1

        k[0,1] = B(-2 * delta[0][1]) / (D(2 * delta[0,0], theta_1) / k[0,0] - x1)

        pose[2,0] = -(A(-2 * delta[0,1]) / k[0,1] - C(2 * delta[0,0], theta_1) / k[0,0] - x1)
        gamma = np.ndarray(shape=(3, 3))
        gamma[2,:] = -copysign(1, pose[3,1])
        s_end = np.ndarray(shape=(3, 3))
        s_end[2,:] = abs(pose[3, 1])
        pose[2, 1] = 0

        alpha = np.ndarray(shape=(2, 1))
        v = np.ndarray(shape=(2, 1))

        for i in range(0, 2):
            gamma[i,:] = copysign(1, delta[i]) * copysign(1, k[i]) # FIXME not sure about scilab "delta(i)" equivalent
            s_end[i,:] = abs(2 * delta[i] / k[i])
            alpha[i,:] = gamma[i] * copysign(1, k[i]) * abs(k[i] / s_end[i])
            if sigma_max / abs(alpha[i]) < v_des:
                v[i,:] = sigma_max / abs(alpha[i])
            else:
                v[i,:] = v_des

        alphas_are_not_well = False
        for i in range(0, 2):
            if alpha[i,0] > alpha_max:
                alphas_are_not_well = True

        length = 2 * sum(s_end) - s_end[2]

        if len(sys.argv) == 4 and mode == 'r':
            descr = []
            descr.append(["StraightLine", [0, 0, 0], pose[2,:], s_end[2,0], v_des])
            for i in range(0, 2):
                descr.append(["ClothoideLine", -gamma[i], alpha[i], s_end[i], 'in', v[i]])
                descr.append(["ClothoideLine", -gamma[i], -alpha[i], s_end[i], 'out', v[i]])
        else:
            descr = []
            for i in range(0, 2):
                descr.append(["ClothoideLine", gamma[i], alpha[i], s_end[i], 'in', v[i]])
                descr.append(["ClothoideLine", gamma[i], -alpha[i], s_end[i], 'out', v[i]])
            descr.append(["StraightLine", [0, 0, 0], pose[2, :], s_end[2, 0], v_des])

        print(gamma) #FIXME WTF ?
        aux_pose = pose[1,:]
        robot_poses = []
        for i in range(0,5):
            poses = calcPathElementPoints(descr[i], aux_pose, 0.05)
            aux_pose = poses[-1,:]
            add_poses = transformCoords(goal_pose, poses)
            robot_poses = [[robot_poses], [add_poses]]

        return descr, length, robot_poses
