from math import pi, sqrt, cos, sin, copysign, pow, atan2
import numpy as np
from TrajectoryStuff_02 import Pose


class AngleFinder:
    def __init__(self):
        self.is_first_time = True
        self.true_angle = 0
        self.last_angle = 0

    def getAngle(self, bad_angle):
        if self.is_first_time:
            self.true_angle = bad_angle
            self.is_first_time = False
        else:
            if -pi < bad_angle - self.last_angle < pi:
                self.true_angle += bad_angle - self.last_angle
            elif pi < bad_angle - self.last_angle:
                self.true_angle += bad_angle - self.last_angle - 2 * pi
            else:
                self.true_angle += bad_angle - self.last_angle + 2 * pi

        self.last_angle = bad_angle

        return self.true_angle


########################################################################################################################

def matmult(X, Y):
    if len(X[0]) != len(Y):
        raise ValueError("Matrices size are not factorable")
    else:
        result = [[0 for i in range(len(Y[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(Y[0])):
                for k in range(len(Y)):
                    result[i][j] += X[i][k] * Y[k][j]
        return result


def matdiv(X, Y):
    if len(X[0]) != len(Y):
        raise ValueError("Matrices size are not factorable")
    else:
        result = [[0 for i in range(len(Y[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(Y[0])):
                for k in range(len(Y)):
                    result[i][j] += X[i][k] / Y[k][j]
        return result


def matdiff(X, Y):
    if (len(X) != len(Y)) or (len(X[0]) != len(Y[0])):
        raise ValueError("Matrices size are not subtractable")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] - Y[i][j]
        return result


def matadd(X, Y):
    if (len(X) != len(Y)) or (len(X[0]) != len(Y[0])):
        raise ValueError("Matrices size are not subtractable")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] + Y[i][j]
        return result


def multmat(X, Y):  # X is matrice, Y is number
    if (type(X) is not list) and (type(Y) is not float or int):
        raise ValueError("Argument types are not compatible")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] * Y
        return result


def divmat(X, Y):  # X is matrice, Y is number
    if (type(X) is not list) and (type(Y) is not float or int):
        raise ValueError("Argument types are not compatible")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] / Y
        return result


########################################################################################################################

def f(x):
    return (1 + 0.926 * x) / (2 + 1.792 * x + 3.104 * x ** 2)


def g(x):
    return 1 / (2 + 4.412 * x + 3.492 * x ** 2 + 6.67 * x ** 3)


def Cf(x):
    return 1 / 2 + f(x) * sin(pi / 2 * x ** 2) - g(x) * cos(pi / 2 * x ** 2)


def Sf(x):
    return 1 / 2 - f(x) * cos(pi / 2 * x ** 2) - g(x) * sin(pi / 2 * x ** 2)


def getClosest(x_r, y_r):
    pass


def xCoord(gamma, alpha, s):
    return gamma * sqrt(pi / abs(alpha)) * Cf(sqrt(abs(alpha) / pi) * s)


def yCoord(gamma, alpha, s):
    return gamma * copysign(1, alpha) * sqrt(pi / abs(alpha)) * Sf(sqrt(abs(alpha) / pi) * s)


def X(b):
    return copysign(1, b) * sqrt(pi * abs(b)) * Cf(sqrt(abs(b) / pi))


def Y(b):
    return sqrt(pi * abs(b)) * Sf(sqrt(abs(b) / pi))


def A(b1, b2=None):
    if b2 is None:
        return X(b1) * (1 + cos(b1)) + Y(b1) * sin(b1)
    else:
        return X(b1) * (1 + cos(b1 + b2)) + Y(b1) * sin(b1 + b2) + sin(0.5 * b1 + b2) - sin(0.5 * b1)


def B(b1, b2=None):
    if b2 is None:
        return X(b1) * sin(b1) + Y(b1) * (1 - cos(b1))
    else:
        return X(b1) * sin(b1 + b2) + Y(b1) * (1 - cos(b1 + b2)) - cos(0.5 * b1 + b2) + cos(0.5 * b1)


def C(b, psi):
    if len(b) == 1:
        return A(b[0]) * cos(psi) - B(b[0]) * sin(psi)
    else:
        return A(b[0], b[1]) * cos(psi) - B(b[0], b[1]) * sin(psi)


def D(b, psi):
    if len(b) == 1:
        return A(b[0]) * sin(psi) + B(b[0]) * cos(psi)
    else:
        return A(b[0], b[1]) * sin(psi) + B(b[0], b[1]) * cos(psi)


def G(b, psi):
    return B(b + psi) + D([b], psi)

########################################################################################################################

def calcClothoidPoints(gamma, alpha, s_end, type, step):
    poses = np.ndarray(shape=(int(s_end // step) + 2, 3))
    if (type == "in"):
        i = 0
        for s in np.arange(0, s_end, step):
            poses[i] = np.array([xCoord(gamma, alpha, s), yCoord(gamma, alpha, s), 0.5 * alpha * pow(s, 2)]).reshape((3))
            i += 1
        poses[i] = np.array([xCoord(gamma, alpha, s_end), yCoord(gamma, alpha, s_end), 0.5 * alpha * pow(s_end, 2)]).reshape((3))
    else:

        origin_pose = Pose(xCoord(-gamma, alpha, s_end), yCoord(-gamma, alpha, s_end), 0.5 * alpha * pow(s_end, 2))
        i = 0
        for s in np.arange(0, s_end, step):
            aux_pose = np.array([xCoord(-gamma, alpha, s_end - s), yCoord(-gamma, alpha, s_end - s),
                        0.5 * alpha * pow(s_end - s, 2)]).reshape((1, 3))
            poses[i] = transformCoords(origin_pose, aux_pose, 'b')
            i += 1
        aux_pose = np.array([xCoord(-gamma, alpha, 0.0), yCoord(-gamma, alpha, 0.0), 0.0]).reshape((1, 3))
        poses[i] = transformCoords(origin_pose, aux_pose, 'b')
    return poses


def calcArcPoints(point_c, point_0, point_1, step):
    k = 1 / point_c[0][1];
    delta = point_1[0][2];
    i = 0
    poses = np.ndarray(shape=(int(abs(delta) // abs(step*k)) + 2, 3))
    for d in np.arange(0, delta, copysign(1, delta)*abs(step*k)):
        poses[i] = [sin(d) / k, (1 - cos(d)) / k, d]
        i += 1
    poses[i] = [sin(delta) / k, (1 - cos(delta)) / k, delta]
    return poses


def calcStraightLinePoints(pose_0, pose_1, s_end, step):
    alpha = atan2(pose_1[0][1] - pose_0[0][1], pose_1[0][0] - pose_0[0][0])
    i = 0
    poses = np.ndarray(shape=(int(s_end // step) + 2, 3))
    for s in np.arange(0, s_end, step):
        poses[i] = [s * cos(alpha), s * sin(alpha), pose_0[0][2]]
        i += 1
    poses[i] = [s_end * cos(alpha), s_end * sin(alpha), pose_0[0][2]]
    return poses


def transformCoords(origin_pose=None, old_poses=None, direction=None):
    T = np.array([[cos(origin_pose.angle), -sin(origin_pose.angle), origin_pose.point.x],
         [sin(origin_pose.angle), cos(origin_pose.angle), origin_pose.point.y],
         [0, 0, 1]])
    new_poses = np.ndarray(shape=(old_poses.shape[0], 3))
    if direction == "b":
        T = np.linalg.inv(T)
        new_poses[:, 2] = old_poses[:, 2] - origin_pose.angle
    else:
        new_poses[:, 2] = old_poses[:, 2] + origin_pose.angle

    aux_poses = T.dot(np.vstack([old_poses[:, 0:2].T, (np.ones(shape=(old_poses[:,0].T.shape)))]))
    new_poses[:, 0:2] = aux_poses[0:2, :].T

    return new_poses


def calcPathElementPoints(element, start_pose, step):
    if element[0] == "ClothoidLine":
        aux_poses = calcClothoidPoints(element[2], element[3], element[4], element[5], step)
    elif element[0] == "StraightLine":
        aux_poses = calcStraightLinePoints(element[1], element[2], element[3], step)
    elif element[0] == "CircleLine":
        aux_poses = calcArcPoints(element[1], element[2], element[3], step)

    return transformCoords(start_pose, aux_poses)
