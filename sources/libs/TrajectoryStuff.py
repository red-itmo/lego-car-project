from math import sin, cos, atan2, sqrt, pi, copysign
from libs.Auxilary import AngleFinder


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y


class TrajectoryPoint:

    def __init__(self, x, dx, ddx, y, dy, ddy):
        self.x, self.dx, self.ddx = x, dx, ddx
        self.y, self.dy, self.ddy = y, dy, ddy

    def getPoint(self):
        return self.x, self.dx, self.ddx, self.y, self.dy, self.ddy


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

    def __init__(self, point_c, point_0, point_1, direction, v=0, accuracy=0):
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
