from math import pi, copysign, sin, cos
import numpy as np
from scipy import optimize
import random
from src.Auxilary import *
from src.TrajectoryStuff_02 import Pose
import matplotlib.pyplot as plt

k_max = 1.5
k_max = 0.9 * k_max
L = 0.165
v_min = 0.05
Omega_max = 10
sigma_max = Omega_max / L
alpha_max = sigma_max / v_min


class EESPlaner:
    def getTrajectory(self, start_pose, goal_pose, v_des, step, reverse=False):
        if reverse:
            init_pose = goal_pose
            goal_pose = start_pose
        else:
            init_pose = start_pose

        pose = np.zeros(shape=(3,3))
        pose[0,:] = transformCoords(goal_pose, np.array([[init_pose.point.x, init_pose.point.y, init_pose.angle]]), 'b')

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

            k[1,0] = B(-2 * delta[1,0]) / (D([2 * delta[0,0]], theta_1) / k[0,0] + y1)

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
            aux_pose = Pose(0.0, 0.0, 0.0)
            descr = []
            descr.append(["StraightLine", np.array([[0.0, 0.0, 0.0]]), np.array([pose[2]]), s_end[2,0], v_des])
            for i in range(1, -1, -1):
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), -gamma[i], -alpha[i], s_end[i], 'in', v[i]])
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), -gamma[i], alpha[i], s_end[i], 'out', v[i]])
        else:
            aux_pose = Pose(pose[0, 0], pose[0, 1], pose[0, 2])
            descr = []
            for i in range(0, 2):
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), gamma[i], alpha[i], s_end[i], 'in', v[i]])
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), gamma[i], -alpha[i], s_end[i], 'out', v[i]])
            descr.append(["StraightLine", np.array([pose[2]]), np.array([[0.0, 0.0, 0.0]]), s_end[2, 0], v_des])

        robot_poses = np.ndarray(shape=(1,3))
        for i in range(0,5):
            poses = calcPathElementPoints(descr[i], aux_pose, step)

            if descr[i][0] == "ClothoidLine":
                descr[i][1] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][1]));
            elif descr[i][0] == "StraightLine":
                descr[i][1] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][1]));
                descr[i][2] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][2]));

            aux_pose = Pose(poses[-1, 0], poses[-1, 1], poses[-1, 2])
            add_poses = transformCoords(goal_pose, poses)
            robot_poses = np.vstack((robot_poses, add_poses))

            # for debugging purposes; may be deleted later
            #np.savetxt("/home/evgeniy/Desktop/test" + str(i) + ".txt", add_poses, fmt='%.5f')

        return descr, length, robot_poses



class eeS_Planer:

    def __init__(self, samples_num=100):
        self.samples_num = samples_num

    def findGlobalMinimaOfG(self, theta_1, y1):
        step  = (pi/2) / self.samples_num
        f = lambda x: abs(G(2*x, theta_1))
        max_x = -pi/2
        for x in np.arange(-pi/2, pi/2, step):
            if f(x) > f(max_x):
                max_x = x

        f = lambda x: -abs(G(2*x, theta_1))
        res = optimize.minimize_scalar(f, bounds=(max_x - step, max_x + step), method='bounded')
        return res.x, -G(2*res.x, theta_1) / y1

    def findSpecialZeroOfG(self, theta_1):
        # print(theta_1)
        f = lambda x: G(2*x, theta_1)
        if theta_1 > 0:
            delta = optimize.brentq(f, -theta_1 / 2, 0)
        elif theta_1 < 0:
            delta = optimize.brentq(f, 0, -theta_1 / 2,)
        else:
            delta = 0

        return delta

    def getTrajectory(self, start_pose, goal_pose, v_des, step, reverse=False):
        if reverse:
            init_pose = goal_pose
            goal_pose = start_pose
        else:
            init_pose = start_pose

        pose = np.zeros(shape=(3,3))
        pose[0,:] = transformCoords(goal_pose, np.array([[init_pose.point.x, init_pose.point.y, init_pose.angle]]), 'b')

        if pose[0,2] > pi:
            pose[0,2] = pose[0,2] - 2 * pi
        elif pose[0,2] < -pi:
            pose[0,2] = pose[0,2] + 2 * pi

        x1 = pose[0,0]
        y1 = pose[0,1]
        theta_1 = pose[0,2]

        delta = np.zeros(shape=(2, 1))
        k = np.zeros(shape=(2, 1))

        if y1 == 0:
            delta[0,0] = self.findSpecialZeroOfG(theta_1)
            k[0,0] = k_max
            # print("in 1")
        else:
            delta_extr, k_found = self.findGlobalMinimaOfG(theta_1, y1)
            if abs(k_found) < k_max:
                delta[0,0] = delta_extr
                k[0,0] = k_found
                # print("in 2")
            else:
                delta_zero = self.findSpecialZeroOfG(theta_1)
                f = lambda x: abs(G(2*x, theta_1) / y1) - k_max
                if delta_zero < delta_extr:
                    res = optimize.brentq(f, delta_zero, delta_extr)
                else:
                    res = optimize.brentq(f, delta_extr, delta_zero)
                delta[0,0] = res
                k[0,0] = copysign(1, k_found)*k_max
                # print("in 3")
                # print(delta_zero, delta_extr)

        delta[1,0] = -delta[0,0] - 0.5 * theta_1
        k[1,0] = -k[0,0]

        # print("delta_1 = ", delta[0,0])
        # print("k_1 = ", k[0,0])

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

        length = 2 * sum(s_end) - s_end[2,0]

        if reverse:
            aux_pose = Pose(0.0, 0.0, 0.0)
            descr = []
            descr.append(["StraightLine", np.array([[0.0, 0.0, 0.0]]), np.array([pose[2]]), s_end[2,0], v_des])
            for i in range(1, -1, -1):
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), -gamma[i], -alpha[i], s_end[i], 'in', v[i]])
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), -gamma[i], alpha[i], s_end[i], 'out', v[i]])
        else:
            aux_pose = Pose(pose[0, 0], pose[0, 1], pose[0, 2])
            descr = []
            for i in range(0, 2):
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), gamma[i], alpha[i], s_end[i], 'in', v[i]])
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), gamma[i], -alpha[i], s_end[i], 'out', v[i]])
            descr.append(["StraightLine", np.array([pose[2]]), np.array([[0.0, 0.0, 0.0]]), s_end[2, 0], v_des])

        robot_poses = np.ndarray(shape=(1,3))
        for i in range(0,5):
            poses = calcPathElementPoints(descr[i], aux_pose, step)

            if descr[i][0] == "ClothoidLine":
                descr[i][1] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][1]));
            elif descr[i][0] == "StraightLine":
                descr[i][1] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][1]));
                descr[i][2] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][2]));

            aux_pose = Pose(poses[-1, 0], poses[-1, 1], poses[-1, 2])
            add_poses = transformCoords(goal_pose, poses)
            robot_poses = np.vstack((robot_poses, add_poses))

            # for debugging purposes; may be deleted later
            #np.savetxt("/home/evgeniy/Desktop/test" + str(i) + ".txt", add_poses, fmt='%.5f')

        return descr, length, robot_poses


class TTSPlaner:
    def getTrajectory(self, start_pose, goal_pose, v_des, step, reverse=False):
        if reverse:
            init_pose = goal_pose
            goal_pose = start_pose
        else:
            init_pose = start_pose

        pose = np.zeros(shape=(3,3))
        pose[0,:] = transformCoords(goal_pose, np.array([[init_pose.point.x, init_pose.point.y, init_pose.angle]]), 'b')

        if pose[0,2] > pi:
            pose[0,2] = pose[0,2] - 2 * pi
        elif pose[0,2] < -pi:
            pose[0,2] = pose[0,2] + 2 * pi

        x1 = pose[0,0]
        y1 = pose[0,1]
        theta_1 = pose[0,2]

        delta = np.zeros(shape=(2, 1))
        k = np.zeros(shape=(2, 1))
        R = np.zeros(shape=(2, 1))

        something_goes_wrong = True
        while something_goes_wrong:
            if 0 <= theta_1 and theta_1 < pi:
                flag = False
                while flag != True:
                    flag = True
                    delta[0,0] = random.uniform(-pi, pi - theta_1)
                    if delta[0,0] == 0:
                        flag = False
            elif -pi <= theta_1 and theta_1 < 0:
                flag = False
                while flag != True:
                    flag = True
                    delta[0,0] = random.uniform(-pi - theta_1, pi)
                    if delta[0,0] == 0:
                        flag = False

            flag = False
            while flag != True:
                flag = True
                k[0,0] = random.uniform(-k_max, k_max)
                if k[0,0] == 0:
                    flag = False

            R[0,0] = random.uniform(0, 1)
            R[1,0] = random.uniform(0, 1)

            delta[1,0] = -delta[0,0] - theta_1
            delta_CC = ((1 - R) * delta) / 2
            delta_C = R * delta

            k[1,0] = B(-2*delta_CC[1,0], -delta_C[1,0]) / (D([2 * delta_CC[0,0], delta_C[0,0]], theta_1) / k[0,0] + y1)

            pose[2,0] = -(A(-2*delta_CC[1,0], -delta_C[1,0]) / k[1,0] - C([2 * delta_CC[0,0], delta_C[0,0]], theta_1) / k[0,0] - x1);

            gamma = np.ndarray(shape=(3, 1))
            gamma[2,:] = -copysign(1, pose[2,0])
            s_end = np.ndarray(shape=(3, 1))
            s_end[2,:] = abs(pose[2, 0])
            pose[2, 1] = 0

            alpha = np.ndarray(shape=(2, 1))
            v = np.ndarray(shape=(2, 1))

            for i in range(0, 2):
                gamma[i,0] = copysign(1, delta_CC[i,0]) * copysign(1, k[i,0])
                s_end[i,0] = abs(2 * delta_CC[i] / k[i])
                alpha[i,0] = gamma[i] * copysign(1, k[i]) * abs(k[i] / s_end[i])
                if sigma_max / abs(alpha[i]) < v_des:
                    v[i,0] = sigma_max / abs(alpha[i])
                else:
                    v[i,0] = v_des

            something_goes_wrong = False
            for i in range(0, 2):
                if alpha[i,0] > alpha_max:
                    something_goes_wrong = True

        length = 2 * sum(s_end) - s_end[2,0] + abs(delta_C[1,0] / k[1,0]) + abs(delta_C[0,0] / k[0,0])

        if reverse:
            aux_pose = Pose(0.0, 0.0, 0.0)
            descr = []
            descr.append(["StraightLine", np.array([[0.0, 0.0, 0.0]]), np.array([pose[2]]), s_end[2,0], v_des])
            for i in range(1, -1, -1):
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), -gamma[i], -alpha[i], s_end[i], 'in', v[i]])
                direction = 'ccw' if -delta_C[i,0] > 0 else 'cw'
                descr.append(["CircleLine", np.array([[0, 1/k[i,0], 0]]), np.array([[0, 0, 0]]), np.array([[sin(-delta_C[i,0]) / k[i,0], (1 - cos(-delta_C[i,0])) / k[i,0], -delta_C[i]]]), direction, v[i]])
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), -gamma[i], alpha[i], s_end[i], 'out', v[i]])
        else:
            aux_pose = Pose(pose[0, 0], pose[0, 1], pose[0, 2])
            descr = []
            for i in range(0, 2):
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), gamma[i], alpha[i], s_end[i], 'in', v[i]])
                direction = 'ccw' if delta_C[i,0] > 0 else 'cw'
                descr.append(["CircleLine", np.array([[0, 1/k[i,0], 0]]), np.array([[0, 0, 0]]), np.array([[sin(delta_C[i,0]) / k[i,0], (1 - cos(delta_C[i,0])) / k[i,0], delta_C[i]]]), direction, v[i]])
                descr.append(["ClothoidLine", np.array([[0, 0, 0]]), gamma[i], -alpha[i], s_end[i], 'out', v[i]])
            descr.append(["StraightLine", np.array([pose[2]]), np.array([[0.0, 0.0, 0.0]]), s_end[2, 0], v_des])

        robot_poses = np.ndarray(shape=(1,3))
        # print(robot_poses)
        for i in range(0,7):
            poses = calcPathElementPoints(descr[i], aux_pose, step)

            if descr[i][0] == "ClothoidLine":
                descr[i][1] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][1]));
            elif descr[i][0] == "StraightLine":
                descr[i][1] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][1]));
                descr[i][2] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][2]));
            elif descr[i][0] == "CircleLine":
                descr[i][1] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][1]));
                descr[i][2] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][2]));
                descr[i][3] = transformCoords(goal_pose, transformCoords(aux_pose, descr[i][3]));

            aux_pose = Pose(poses[-1, 0], poses[-1, 1], poses[-1, 2])
            add_poses = transformCoords(goal_pose, poses)
            robot_poses = np.vstack((robot_poses, add_poses))
            # if i == 0:
            #     print(robot_poses)

            # for debugging purposes; may be deleted later
            ##np.savetxt("/home/evgeniy/Desktop/test" + str(i) + ".txt", add_poses, fmt='%.5f')

        return descr, length, robot_poses


# for debugging purposes; may be deleted later
if __name__=="__main__":
    # 0.22500000000000003 0.22500000000000003 0
    # 1.2438573108513324 0.22500000000000003 0
    X, Y, Z = eeS_Planer().getTrajectory(Pose(0.22500000000000003, 0.22500000000000003, 0), Pose(1.2438573108513324, 0.22500000000000003, 0.0), 0.2, 0.01)
    # d = eeS_Planer().findSpecialZeroOfG(3)
    # f = lambda x: G(2*x, -4)
    # delta = optimize.brentq(f, 0, 2)
    # print("d = ", d)
    #print(Z)
    #Z = Z[1:]
    #f = lambda x: G(2*x, theta_1)
    #theta = 3

    plt.plot(Z[1:,0],Z[1:,1])
    plt.show()
