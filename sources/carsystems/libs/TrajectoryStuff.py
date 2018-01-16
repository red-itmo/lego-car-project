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


class ParkingLine(TrajectoryLine):

    def __init__(self, point_1, point_l, point_r, radius, depth, delta_1, delta_2, v):

        super(ParkingLine, self).__init__()

        # some obvious calculations
        point_4 = Point(point_r.x - delta_2, point_r.y + delta_2)
        point_6 = Point(point_l.x + delta_1, point_l.y - depth)
        point_7 = Point(point_6.x, point_6.y + radius)

        # some other calculations
        point_5 = self.findPoint5(point_7, point_4, radius)
        point_8, point_3 = self.findPoints8And3(point_1, point_4, point_5, radius)
        point_2 = Point(point_8.x, point_1.y);

        # pieces definition
        self.horiz_line = StraightLine(point_1, point_2, v)
        self.arc_first = CircleLine(point_8, point_2, point_3, v)
        self.inclin_line = StraightLine(point_3, point_5, v)
        self.arc_final = CircleLine(point_7, point_5, point_6, v)

        # time calculation and some other things
        self.end_time = sum([x.end_time for x in [self.horiz_line, self.arc_first, self.inclin_line, self.arc_final]])
        self.time_for = {"point_2": self.horiz_line.end_time, "point_3": self.horiz_line.end_time + self.arc_first.end_time, "point_5": self.end_time - self.arc_final.end_time}
        self.point_6 = point_6


    def findPoint5(self, point_7, point_4, radius):
        """It finds coordinates of point_5 using Newton's method"""
        e = 1
        alpha = pi/4
        while abs(e) > 0.001:
            deriv_e = -(point_4.x - point_7.x) / cos(alpha)**2 + radius * sin(alpha) / cos(alpha)**2
            e = (point_4.y - point_7.y) - (point_4.x - point_7.x) * tan(alpha) + radius * cos(alpha) + radius * tan(alpha) * sin(alpha)
            alpha -= e / deriv_e
        return Point(point_7.x + radius * sin(alpha), point_7.y - radius * cos(alpha))


    def findPoints8And3(self, point_1, point_4, point_5, radius):
        k = (point_4.y - point_5.y) / (point_4.x - point_5.x)
        alpha = atan(k)
        b = point_4.y - k * point_4.x
        h = radius / cos(alpha)
        point_8 = Point(1.0 / k * (point_1.y - radius - b + h), point_1.y - radius)
        point_3 = Point(point_8.x - radius * cos(alpha), point_8.y + radius * sin(alpha))
        return point_8, point_3


    def getCoordinates(self, t):
        if t > self.end_time:
            self.is_end = True
            return TrajectoryPoint(self.point_6.x, 0.0, 0.0, self.point_6.y, 0.0, 0.0)
        elif t > self.time_for["point_5"]:
            return self.arc_final.getCoordinates(t - self.time_for["point_5"])
        elif t > self.time_for["point_3"]:
            return self.inclin_line.getCoordinates(t - self.time_for["point_3"])
        elif t > self.time_for["point_2"]:
            return self.arc_first.getCoordinates(t - self.time_for["point_2"])
        else:
            return self.horiz_line.getCoordinates(t)
