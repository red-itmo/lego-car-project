from math import sin, cos, tan, atan, atan2, sqrt, pi, copysign


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

    def __init__(self, point_0, point_1, v):
        super(StraightLine, self).__init__()
        self.point_0, self.point_1 = point_0, point_1
        gamma = atan2(point_1.y - point_0.y, point_1.x - point_0.x)
        self.vx = v * cos(gamma)
        self.vy = v * sin(gamma)
        self.end_time = sqrt( (point_1.y - point_0.y)**2 + (point_1.x - point_0.x)**2 ) / v

    def getCoordinates(self, t):
        if t > self.end_time:
            self.is_end = True
            return TrajectoryPoint(self.point_1.x, 0.0, 0.0, self.point_1.y, 0.0, 0.0)
        else:
            x_r = self.vx * t + self.point_0.x
            y_r = self.vy * t + self.point_0.y
            return TrajectoryPoint(x_r, self.vx, 0.0, y_r, self.vy, 0.0)


class CircleLine(TrajectoryLine):

    def __init__(self, point_c, point_0, point_1, v):
        super(CircleLine, self).__init__()
        self.point_c, self.point_1 = point_c, point_1
        self.angle_0 = atan2(point_0.y - point_c.y, point_0.x - point_c.x)
        angle_1 = atan2(point_1.y - point_c.y, point_1.x - point_c.x)
        delta_angle = angle_1 - self.angle_0
        if delta_angle > pi:
            delta_angle = delta_angle - 2 * pi;
        elif delta_angle < -pi:
            delta_angle = delta_angle + 2 * pi;
        self.radius = sqrt( (point_0.y - point_c.y)**2 + (point_0.x - point_c.x)**2 )
        self.omega = copysign(1, delta_angle) * v / self.radius
        self.end_time = abs(delta_angle * self.radius) / v

    def getCoordinates(self, t):
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
