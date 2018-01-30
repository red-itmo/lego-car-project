from math import sin, cos, atan2, sqrt, pi, copysign
from Auxilary import *
import numpy as np

class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y


class TrajectoryPoint:

    def __init__(self, x, dx, ddx, y, dy, ddy):
        self.x, self.dx, self.ddx = x, dx, ddx
        self.y, self.dy, self.ddy = y, dy, ddy

    def __str__(self):
        return "{0} {1} {2} {3} {4} {5}".format(self.x, self.y, self.dx, self.dy, self.ddx, self.ddy)

    def getPoint(self):
        return self.x, self.dx, self.ddx, self.y, self.dy, self.ddy


class Pose:

    def __init__(self, x, y, angle):
        self.point = Point(x, y)
        self.angle = angle


class TrajectoryLine:

    def __init__(self):
        self.is_end = False

    def isEnd(self):
        return self.is_end


class StraightLine(TrajectoryLine):

    def __init__(self, point_0, point_1, v=0, accuracy=0):
        super(StraightLine, self).__init__()
        self.gamma = atan2(point_1.y - point_0.y, point_1.x - point_0.x)
        self.point_0, self.point_1 = point_0, point_1
        if v != 0:
            self.vx = v * cos(self.gamma)
            self.vy = v * sin(self.gamma)
            self.end_time = sqrt( (point_1.y - point_0.y)**2 + (point_1.x - point_0.x)**2) / v
        else:
            self.accuracy = accuracy

    def getClosest(self, x_r, y_r):
        x_r_new = x_r * cos(self.gamma) + y_r * sin(self.gamma)
        if x_r_new < 0:
            return 0
        else:
            return x_r_new

    def getCoordinatesDistance(self, x_r, y_r):
        if sqrt( (self.point_1.x - x_r) ** 2 + (self.point_1.y - y_r) ** 2) < self.accuracy:
            self.is_end = True
        s = self.getClosest(x_r,  y_r)
        x_ref = self.point_0.x + s * cos(self.gamma)
        y_ref = self.point_0.y + s * sin(self.gamma)
        kappa = 0
        t_hat = cos(self.gamma), sin(self.gamma)
        return x_ref, y_ref, kappa, t_hat

    def getCoordinatesTime(self, t):
        if t > self.end_time:
            self.is_end = True
            return TrajectoryPoint(self.point_1.x, 0.0, 0.0, self.point_1.y, 0.0, 0.0)
        else:
            x_r = self.vx * t + self.point_0.x
            y_r = self.vy * t + self.point_0.y
            return TrajectoryPoint(x_r, self.vx, 0.0, y_r, self.vy, 0.0)


class CircleLine(TrajectoryLine):

    def __init__(self, point_c, point_0, point_1, direction, v=0.0, accuracy=0):
        super(CircleLine, self).__init__()
        self.angle_finder = AngleFinder()
        self.point_c, self.point_1 = point_c, point_1
        self.angle_0 = atan2(point_0.y - point_c.y, point_0.x - point_c.x)
        self.angle_1 = atan2(point_1.y - point_c.y, point_1.x - point_c.x)
        self.radius = sqrt((point_0.y - point_c.y) ** 2 + (point_0.x - point_c.x) ** 2)
        self.direction = direction

        if v != 0:
            delta_angle = self.angle_1 - self.angle_0
            if self.direction == "cw" and delta_angle > 0:
                delta_angle = delta_angle - 2 * pi
            elif self.direction == "ccw" and delta_angle < 0:
                delta_angle = delta_angle + 2 * pi
            self.omega = copysign(1, delta_angle) * v / self.radius
            self.end_time = abs(delta_angle * self.radius) / v
        else:
            self.accuracy = accuracy

    def getClosest(self, x_r, y_r):
        angle_r = self.angle_finder.getAngle(atan2(y_r - self.point_c.y, x_r - self.point_c.x))

        d_alpha = angle_r - self.angle_0
        if d_alpha >= 0:
            return 2 * pi * self.radius - d_alpha * self.radius if self.direction == "cw" else d_alpha * self.radius
        else:
            return abs(d_alpha * self.radius) if self.direction == "cw" else 2 * pi * self.radius + d_alpha * self.radius

    def getCoordinatesDistance(self, x_r, y_r):
        if sqrt((self.point_1.x - x_r) ** 2 + (self.point_1.y - y_r) ** 2) < self.accuracy:
            self.is_end = True
        s = self.getClosest(x_r,  y_r)
        alpha = -s / self.radius if self.direction == "cw" else s / self.radius
        x_ref = self.point_c.x + self.radius * cos(alpha + self.angle_0)
        y_ref = self.point_c.y + self.radius * sin(alpha + self.angle_0)
        kappa = -1 / self.radius if self.direction == "cw" else 1 / self.radius
        t_hat = -sin(alpha + self.angle_0), cos(alpha + self.angle_0)
        return x_ref, y_ref, kappa, t_hat

    def getCoordinatesTime(self, t):
        if t > self.end_time:
            self.is_end = True
            return TrajectoryPoint(self.point_1.x, 0.0, 0.0, self.point_1.y, 0.0, 0.0)
        else:
            phase = self.angle_0 + self.omega * t
            x_r = self.point_c.x + self.radius * cos(phase)
            y_r = self.point_c.y + self.radius * sin(phase)
            vx_r = -self.radius * self.omega * sin(phase)
            vy_r = self.radius * self.omega * cos(phase)
            ax_r = -self.radius * self.omega**2 * cos(phase)
            ay_r = -self.radius * self.omega**2 * sin(phase)
            return TrajectoryPoint(x_r, vx_r, ax_r, y_r, vy_r, ay_r)


class ClothoidLine(TrajectoryLine):
    def __init__(self, pose_0, gamma, alpha, s_end,  v=0.0, accuracy=0, type=None):
        super(ClothoidLine, self).__init__()
        self.gamma, self.alpha, self.s_end, self.type = gamma, alpha, s_end, type
        self.delta = (self.alpha * self.s_end ** 2) / 2


        self.R1 = np.array([[cos(pose_0.angle), -sin(pose_0.angle)], [sin(pose_0.angle), cos(pose_0.angle)]])

        self.T1 = np.array([[cos(pose_0.angle), -sin(pose_0.angle), pose_0.point.x],
                            [sin(pose_0.angle), cos(pose_0.angle), pose_0.point.y],
                            [0, 0, 1]])

        if type == "out":
            self.start_point = Point(xCoord(-self.gamma, self.alpha, self.s_end), yCoord(-self.gamma, self.alpha, self.s_end))
            self.R2 = np.array([[cos(self.delta), -sin(self.delta)],
                                [sin(self.delta), cos(self.delta)]])
            self.T2 = np.array([[cos(self.delta), -sin(self.delta), self.start_point.x],
                                [sin(self.delta), cos(self.delta), self.start_point.y],
                                [0, 0, 1]])
            self.R2_inv = np.linalg.inv(self.R2)
            self.T2_inv = np.linalg.inv(self.T2)

        self.speed = v
        if self.speed != 0:
            self.t_end = self.s_end / self.speed
        else:
            self.accuracy = accuracy

    # def getCoordinatesDistance(self, x_r, y_r):
    #     if self.flag == 0:
    #         if (x_r - self.point_1.x) ** 2 + (y_r - self.point_1.y) ** 2 < self.accuracy ** 2:
    #             self.flag = 1
    #
    #     s = self.getClosest(x_r, y_r)
    #
    #     if self.flag == 0:
    #         x_ref = self.xCoord(self.gamma, self.alpha, s)
    #         y_ref = self.yCoord(self.gamma, self.alpha, s)
    #         kappa = self.alpha * s
    #         t_hat = [[self.gamma * cos(abs(self.alpha) * s ** 2 / 2)],
    #                  [self.gamma * copysign(1, self.alpha) * sin(abs(self.alpha) * s ** 2 / 2)]]
    #
    #         return x_ref, y_ref, kappa, t_hat
    #
    #     else:
    #         if (x_r - self.final.x) ** 2 + (y_r - self.final.y) ** 2 < self.accuracy ** 2:
    #             self.is_end = True
    #         x_ref_aux = self.xCoord(-self.gamma, self.alpha, 2 * self.s_end - s)
    #         y_ref_aux = self.yCoord(self.gamma, self.alpha, 2 * self.s_end - s)
    #         kappa = self.alpha * (2 * self.s_end - s)
    #         t_hat_aux = [[self.gamma * cos(abs(self.alpha) / 2 * (2 * self.s_end - s) ** 2)], [
    #             -self.gamma * copysign(1, self.gamma) * sin(abs(self.alpha) / 2 * (2 * self.s_end - s) ** 2)]]
    #         refs = dott(dott(self.T1, self.T2_inv), [[x_ref_aux], [y_ref_aux], [1]])
    #         t_hat = dott(dott(self.R1, self.R2_inv), t_hat_aux)
    #
    #         return refs[0], refs[1], kappa, t_hat

    def getCoordinatesTime(self, t):
        if t > self.t_end:
            self.is_end = True

        if self.type == "in":
            x_ref_aux = xCoord(self.gamma, self.alpha, self.speed * t)
            y_ref_aux = yCoord(self.gamma, self.alpha, self.speed * t)

            x_ref_deriv_aux = self.gamma * self.speed * cos(abs(self.alpha) * (self.speed * t) ** 2 / 2)
            y_ref_deriv_aux = self.gamma * self.speed * copysign(1, self.alpha) * sin(
                abs(self.alpha) * (self.speed * t) ** 2 / 2)

            x_ref_d_deriv_aux = - self.gamma * t * abs(self.alpha) * self.speed ** 3 * sin(
                abs(self.alpha) * (self.speed * t) ** 2 / 2)
            y_ref_d_deriv_aux = self.gamma * t * abs(self.alpha) * self.speed ** 3 * copysign(1, self.alpha) * cos(
                abs(self.alpha) * (self.speed * t) ** 2 / 2)


            refs_d_deriv_aux = np.dot(self.R1, [[x_ref_d_deriv_aux], [y_ref_d_deriv_aux]])
            refs_deriv_aux = np.dot(self.R1, [[x_ref_deriv_aux], [y_ref_deriv_aux]])
            refs_aux = np.dot(self.T1, [[x_ref_aux], [y_ref_aux], [1]])

            return TrajectoryPoint(refs_aux[0][0], refs_deriv_aux[0][0], refs_d_deriv_aux[0][0], refs_aux[1][0],
                                   refs_deriv_aux[1][0], refs_d_deriv_aux[1][0])

        else:
            x_ref_aux = xCoord(-self.gamma, self.alpha, self.s_end - self.speed * t)
            y_ref_aux = yCoord(-self.gamma, self.alpha, self.s_end - self.speed * t)

            x_ref_aux_deriv = self.gamma * self.speed * cos(
                abs(self.alpha) * (self.s_end - self.speed * t) ** 2 / 2)
            y_ref_aux_deriv = self.gamma * self.speed * copysign(1, self.alpha) * sin(
                abs(self.alpha) * (self.s_end - self.speed * t) ** 2 / 2)

            x_ref_aux_d_deriv = self.gamma * abs(self.alpha) * self.speed ** 2 * (self.s_end - self.speed * t) * sin(abs(self.alpha) * (self.s_end - self.speed * t) ** 2 / 2)
            y_ref_aux_d_deriv = -self.gamma * abs(self.alpha) * copysign(1, self.alpha) * self.speed ** 2 * (self.s_end - self.speed * t) * cos(abs(self.alpha) * (self.s_end - self.speed * t) ** 2 / 2)


            refs_d_deriv = np.dot(np.dot(self.R1,self.R2_inv), [[x_ref_aux_d_deriv], [y_ref_aux_d_deriv]])
            refs_deriv = np.dot(np.dot(self.R1, self.R2_inv), [[x_ref_aux_deriv], [y_ref_aux_deriv]])

            refs = np.dot(np.dot(self.T1,self.T2_inv),[[x_ref_aux], [y_ref_aux], [1]])

            return TrajectoryPoint(refs[0][0], refs_deriv[0][0], refs_d_deriv[0][0], refs[1][0], refs_deriv[1][0], refs_d_deriv[1][0])
