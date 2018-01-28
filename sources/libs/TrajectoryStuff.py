from math import sin, cos, atan2, sqrt, pi, copysign, radians
from libs.Auxilary import AngleFinder, matmult


# for debugging purposes (may be deleted later)
PATH = "/Users/osouzdalev/Desktop/RED/lego-car-movement-control/sources/libs/"


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


class AcademicClothoidLine(TrajectoryLine):
    def __init__(self, point_1, v=0.0, accuracy=0):
        super(AcademicClothoidLine, self).__init__()
        self.point_1 = point_1
        self.gamma = 1 if self.point_1.x > 0 else -1
        self.beta = 1 if self.point_1.y > 0 else -1
        self.alpha_sign = 1 if self.point_1.x * self.point_1.y > 0 else -1
        self.alpha, self.s_end = self.find_alpha_and_s_end()
        self.delta = (self.alpha * self.s_end ** 2) / 2
        self.flag = 0
        self.R1 = [[cos(self.delta), -sin(self.delta)], [sin(self.delta), cos(self.delta)]]
        self.R2_inv = [[cos(self.delta), -sin(self.delta)], [sin(self.delta), cos(self.delta)]]

        self.T1 = [[cos(self.delta), -sin(self.delta), self.point_1.x],
                   [sin(self.delta), cos(self.delta), self.point_1.y],
                   [0, 0, 1]]
        self.T2_inv = [
            [cos(self.delta), -sin(self.delta), self.point_1.y * sin(self.delta) + self.point_1.x * cos(self.delta)],
            [sin(self.delta), cos(self.delta), -self.point_1.y * cos(self.delta) + self.point_1.x * sin(self.delta)],
            [0, 0, 1]]
        self.final = matmult(matmult(self.T1, self.T2_inv), [[0], [0], [1]])
        self.speed = v
        if self.speed != 0:
            self.t_end = 2 * self.s_end / self.speed
        else:
            self.accuracy = accuracy

        # for debugging purposes (may be deleted later)
        self.f1 = open(PATH + "first.txt", 'w')
        self.f2 = open(PATH + "second.txt", 'w')

    def __delete__(self):
        self.f1.close()
        self.f2.close()

    def f(self, x):
        return (1 + 0.926 * x) / (2 + 1.792 * x + 3.104 * x ** 2)

    def g(self, x):
        return 1 / (2 + 4.412 * x + 3.492 * x ** 2 + 6.67 * x ** 3)

    def Cf(self, x):
        return 1 / 2 + self.f(x) * sin (pi / 2 * x ** 2 ) - self.g(x) * cos(pi / 2 * x ** 2)

    def Sf(self, x):
        return 1 / 2 - self.f(x) * cos(pi / 2 * x ** 2) - self.g(x) * sin(pi / 2 * x ** 2)

    def find_alpha_and_s_end(self):
        # finding s_0
        k = self.point_1.y / self.point_1.x
        e = copysign(1, self.beta)
        s_0 = x = 0
        while copysign(1, self.beta) * e > 0:
            s_0 += 0.1
            x = self.xCoord(self.gamma, self.alpha_sign, s_0)
            y = self.yCoord(self.gamma, self.alpha_sign, s_0)
            e = k * x - y

        # finding alpha and s
        while abs(e) > 0.001:
            x = self.xCoord(self.gamma, self.alpha_sign, s_0)
            y = self.yCoord(self.gamma, self.alpha_sign, s_0)
            e = k * x - y
            deriv_e = self.gamma * (k * cos(s_0 ** 2 / 2) - copysign(1, self.alpha_sign) * sin(s_0 ** 2 / 2))
            s_0 = s_0 - 0.1 * e * deriv_e

        phi = s_0 ** 2 / 2
        alpha = (x / self.point_1.x) ** 2 * copysign(1, self.alpha_sign)
        s = sqrt(2 * phi / abs(alpha))

        return alpha, s

    def getClosest(self, x_r, y_r):
        pass

    def xCoord(self, gamma, alpha, s):
        return gamma * sqrt(pi / abs(alpha)) * self.Cf(sqrt(abs(alpha) / pi) * s)

    def yCoord(self, gamma, alpha, s):
        return gamma * copysign(1, alpha) * sqrt(pi / abs(alpha)) * self.Sf(sqrt(abs(alpha) / pi) * s)

    def getCoordinatesDistance(self, x_r, y_r):
        if self.flag == 0:
            if (x_r - self.point_1.x) ** 2 + (y_r - self.point_1.y) ** 2 < self.accuracy ** 2:
                self.flag = 1

        s = self.getClosest(x_r, y_r)

        if self.flag == 0:
            x_ref = self.xCoord(self.gamma, self.alpha, s)
            y_ref = self.yCoord(self.gamma, self.alpha, s)
            kappa = self.alpha * s
            t_hat = [[self.gamma * cos(abs(self.alpha) * s ** 2 / 2)], [self.gamma * copysign(1, self.alpha) * sin(abs(self.alpha) * s ** 2 / 2)]]

            return x_ref, y_ref, kappa, t_hat

        else:
            if (x_r - self.final.x) ** 2 + (y_r - self.final.y) ** 2 < self.accuracy ** 2:
                self.is_end = True
            x_ref_aux = self.xCoord(-self.gamma, self.alpha, 2 * self.s_end - s)
            y_ref_aux = self.yCoord(self.gamma, self.alpha, 2 * self.s_end - s)
            kappa = self.alpha * (2 * self.s_end - s)
            t_hat_aux = [[self.gamma * cos(abs(self.alpha) / 2 * (2 * self.s_end - s) ** 2)], [-self.gamma * copysign(1, self.gamma) * sin(abs(self.alpha) / 2 * (2 * self.s_end - s) ** 2)]]
            refs = matmult(matmult(self.T1, self.T2_inv), [[x_ref_aux], [y_ref_aux], [1]])
            t_hat = matmult(matmult(self.R1, self.R2_inv), t_hat_aux)

            return  refs[0], refs[1], kappa, t_hat

    def getCoordinatesTime(self, t):
        if t > self.t_end:
            self.is_end = True

        if t < self.s_end / self.speed:
            x_ref = self.xCoord(self.gamma, self.alpha, self.speed * t)
            y_ref = self.yCoord(self.gamma, self.alpha, self.speed * t)

            x_ref_deriv = self.gamma * self.speed * cos(abs(self.alpha) * (self.speed * t) ** 2 / 2)
            y_ref_deriv = self.gamma * self.speed * copysign(1, self.alpha) * sin(abs(self.alpha) * (self.speed * t) ** 2 / 2)

            x_ref_d_deriv = - self.gamma * t * abs(self.alpha) * self.speed ** 3 * sin(abs(self.alpha) * (self.speed * t) ** 2 / 2)
            y_ref_d_deriv = self.gamma * t * abs(self.alpha) * self.speed ** 3 * copysign(1, self.alpha) * cos(abs(self.alpha) * (self.speed * t) ** 2 / 2)

            # for debugging purposes (may be deleted later)
            output_point = TrajectoryPoint(x_ref, x_ref_deriv, x_ref_d_deriv, y_ref, y_ref_deriv, y_ref_d_deriv)
            self.f1.write("{0} {1}\n".format(t, output_point))

            return TrajectoryPoint(x_ref, x_ref_deriv, x_ref_d_deriv, y_ref, y_ref_deriv, y_ref_d_deriv)

        else:
            x_ref_aux = self.xCoord(-self.gamma, -self.alpha, 2 * self.s_end - self.speed * t)
            y_ref_aux = self.yCoord(-self.gamma, -self.alpha, 2 * self.s_end - self.speed * t)

            x_ref_aux_deriv = self.gamma * self.speed * cos(abs(self.alpha) * (2 * self.s_end - self.speed * t) ** 2 / 2)
            y_ref_aux_deriv = -self.gamma * self.speed * copysign(1, self.alpha) * sin(abs(self.alpha) * (2 * self.s_end - self.speed * t) ** 2 / 2)

            x_ref_aux_d_deriv = self.gamma * abs(self.alpha) * self.speed ** 2 * (2 * self.s_end - self.speed * t) * sin(abs(self.alpha) * (2 * self.s_end - self.speed * t) ** 2 / 2)
            y_ref_aux_d_deriv = self.gamma * abs(self.alpha) * copysign(1, self.alpha) * self.speed ** 2 * (2 * self.s_end - self.speed * t) * cos(abs(self.alpha) * (2 * self.s_end - self.speed * t) ** 2 / 2)

            refs_d_deriv = matmult(matmult(self.R1, self.R2_inv), [[x_ref_aux_d_deriv], [y_ref_aux_d_deriv]])
            refs_deriv = matmult(matmult(self.R1, self.R2_inv), [[x_ref_aux_deriv], [y_ref_aux_deriv]])
            refs = matmult(matmult(self.T1, self.T2_inv), [[x_ref_aux], [y_ref_aux], [1]])

            # for debugging purposes (may be deleted later)
            output_point = TrajectoryPoint(refs[0][0], refs_deriv[0][0], refs_d_deriv[0][0], refs[1][0], refs_deriv[1][0], refs_d_deriv[1][0])
            self.f2.write("{0} {1}\n".format(t, output_point))

            return TrajectoryPoint(refs[0][0], refs_deriv[0][0], refs_d_deriv[0][0], refs[1][0], refs_deriv[1][0], refs_d_deriv[1][0])


class ClothoidLine(TrajectoryLine):
    def __init__(self, pose_0, gamma, alpha, s_end,  v=0.0, accuracy=0, type=None):
        super(ClothoidLine, self).__init__()
        self.gamma, self.alpha, self.s_end, self.type = gamma, alpha, s_end, type
        self.delta = (self.alpha * self.s_end ** 2) / 2

        self.R1 = [[cos(pose_0.angle), -sin(pose_0.angle)], [sin(pose_0.angle), cos(pose_0.angle)]]

        self.T1 = [[cos(pose_0.angle), -sin(pose_0.angle), pose_0.point.x],
                   [sin(pose_0.angle), cos(pose_0.angle), pose_0.point.y],
                   [0, 0, 1]]

        if type == "out":
            self.start_point = Point(self.xCoord(-self.gamma, self.alpha, self.s_end), self.yCoord(-self.gamma, self.alpha, self.s_end))
            self.R2_inv = [[cos(self.delta), sin(self.delta)], [-sin(self.delta), cos(self.delta)]]
            self.T2_inv = [
                [cos(self.delta), sin(self.delta), -self.start_point.y * sin(self.delta) - self.start_point.x * cos(self.delta)],
                [-sin(self.delta), cos(self.delta), -self.start_point.y * cos(self.delta) + self.start_point.x * sin(self.delta)],
                [0, 0, 1]]

        self.speed = v
        if self.speed != 0:
            self.t_end = self.s_end / self.speed
        else:
            self.accuracy = accuracy

        # for debugging purposes (may be deleted later)
        self.f1 = open(PATH + "first.txt", 'w')
        self.f2 = open(PATH + "second.txt", 'w')

    def __delete__(self):
        self.f1.close()
        self.f2.close()

    def f(self, x):
        return (1 + 0.926 * x) / (2 + 1.792 * x + 3.104 * x ** 2)

    def g(self, x):
        return 1 / (2 + 4.412 * x + 3.492 * x ** 2 + 6.67 * x ** 3)

    def Cf(self, x):
        return 1 / 2 + self.f(x) * sin(pi / 2 * x ** 2) - self.g(x) * cos(pi / 2 * x ** 2)

    def Sf(self, x):
        return 1 / 2 - self.f(x) * cos(pi / 2 * x ** 2) - self.g(x) * sin(pi / 2 * x ** 2)

    def getClosest(self, x_r, y_r):
        pass

    def xCoord(self, gamma, alpha, s):
        return gamma * sqrt(pi / abs(alpha)) * self.Cf(sqrt(abs(alpha) / pi) * s)

    def yCoord(self, gamma, alpha, s):
        return gamma * copysign(1, alpha) * sqrt(pi / abs(alpha)) * self.Sf(sqrt(abs(alpha) / pi) * s)

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
    #         refs = matmult(matmult(self.T1, self.T2_inv), [[x_ref_aux], [y_ref_aux], [1]])
    #         t_hat = matmult(matmult(self.R1, self.R2_inv), t_hat_aux)
    #
    #         return refs[0], refs[1], kappa, t_hat

    def getCoordinatesTime(self, t):
        if t > self.t_end:
            self.is_end = True

        if self.type == "in":
            x_ref_aux = self.xCoord(self.gamma, self.alpha, self.speed * t)
            y_ref_aux = self.yCoord(self.gamma, self.alpha, self.speed * t)

            x_ref_deriv_aux = self.gamma * self.speed * cos(abs(self.alpha) * (self.speed * t) ** 2 / 2)
            y_ref_deriv_aux = self.gamma * self.speed * copysign(1, self.alpha) * sin(
                abs(self.alpha) * (self.speed * t) ** 2 / 2)

            x_ref_d_deriv_aux = - self.gamma * t * abs(self.alpha) * self.speed ** 3 * sin(
                abs(self.alpha) * (self.speed * t) ** 2 / 2)
            y_ref_d_deriv_aux = self.gamma * t * abs(self.alpha) * self.speed ** 3 * copysign(1, self.alpha) * cos(
                abs(self.alpha) * (self.speed * t) ** 2 / 2)

            refs_d_deriv_aux = matmult(self.R1, [[x_ref_d_deriv_aux], [y_ref_d_deriv_aux]])
            refs_deriv_aux = matmult(self.R1, [[x_ref_deriv_aux], [y_ref_deriv_aux]])
            refs_aux = matmult(self.T1, [[x_ref_aux], [y_ref_aux], [1]])

            # for debugging purposes (may be deleted later)
            output_point = TrajectoryPoint(x_ref_aux, x_ref_deriv_aux, x_ref_d_deriv_aux, y_ref_aux, y_ref_deriv_aux,
                                           y_ref_d_deriv_aux)
            self.f1.write("{0} {1}\n".format(t, output_point))

            return TrajectoryPoint(refs_aux[0][0], refs_deriv_aux[0][0], refs_d_deriv_aux[0][0], refs_aux[1][0],
                                   refs_deriv_aux[1][0], refs_d_deriv_aux[1][0])

        else:
            x_ref_aux = self.xCoord(-self.gamma, self.alpha, self.s_end - self.speed * t)
            y_ref_aux = self.yCoord(-self.gamma, self.alpha, self.s_end - self.speed * t)

            x_ref_aux_deriv = self.gamma * self.speed * cos(
                abs(self.alpha) * (self.s_end - self.speed * t) ** 2 / 2)
            y_ref_aux_deriv = self.gamma * self.speed * copysign(1, self.alpha) * sin(
                abs(self.alpha) * (self.s_end - self.speed * t) ** 2 / 2)

            x_ref_aux_d_deriv = self.gamma * abs(self.alpha) * self.speed ** 2 * (self.s_end - self.speed * t) * sin(abs(self.alpha) * (self.s_end - self.speed * t) ** 2 / 2)
            y_ref_aux_d_deriv = -self.gamma * abs(self.alpha) * copysign(1, self.alpha) * self.speed ** 2 * (self.s_end - self.speed * t) * cos(abs(self.alpha) * (self.s_end - self.speed * t) ** 2 / 2)

            refs_d_deriv = matmult(matmult(self.R1, self.R2_inv), [[x_ref_aux_d_deriv], [y_ref_aux_d_deriv]])
            refs_deriv = matmult(matmult(self.R1, self.R2_inv), [[x_ref_aux_deriv], [y_ref_aux_deriv]])
            refs = matmult(matmult(self.T1, self.T2_inv), [[x_ref_aux], [y_ref_aux], [1]])


            # for debugging purposes (may be deleted later)
            output_point = TrajectoryPoint(refs[0][0], refs_deriv[0][0], refs_d_deriv[0][0], refs[1][0], refs_deriv[1][0], refs_d_deriv[1][0])
            self.f2.write("{0} {1}\n".format(t, output_point))

            return TrajectoryPoint(refs[0][0], refs_deriv[0][0], refs_d_deriv[0][0], refs[1][0], refs_deriv[1][0], refs_d_deriv[1][0])





# for debugging purposes (may be deleted later);
# NOTE: delete "libs." before "Auxilary" in the imports
# before launching the command: python3 TrajectoryStuff.py
if __name__ == "__main__":

    trajectory_1 = ClothoidLine(Pose(0, 0, 0), 1, -0.118198, 4.892135, 0.2, type="in")
    trajectory_2 = ClothoidLine(Pose(4, -2, -1.4144176), 1, 0.118198, 4.892135, 0.2, type="out")
    trajectory_3 = AcademicClothoidLine(Point(4, -2), 0.2)
    t = 0.0
    f1 = open(PATH + "f1.txt", 'w')
    f2 = open(PATH + "f2.txt", 'w')
    f3 = open(PATH + "f3.txt", 'w')
    while t < 24:
        trajectory_point = trajectory_1.getCoordinatesTime(t)
        f1.write("{0} {1}\n".format(t, trajectory_point))
        trajectory_point = trajectory_2.getCoordinatesTime(t)
        f2.write("{0} {1}\n".format(t, trajectory_point))
        t += 0.1
    t = 0
    while t < 48:
         trajectory_point = trajectory_3.getCoordinatesTime(t)
         f3.write("{0} {1}\n".format(t, trajectory_point))
         t += 0.1

    f1.close()
    f2.close()
    f3.close()
