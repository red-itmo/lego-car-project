#!/usr/bin/env python3
from math import sin, cos


class TrajectoryController:

    def __init__(self):
        self.error_file = open('trajectory_errors.txt', 'w')

    def __delete__(self):
        self.error_file.close()


class ControllerWithLinearization(TrajectoryController):

    # controller coefficients
    Kp_1 = 5.76
    Kd_1 = 4.8
    Kp_2 = 5.76
    Kd_2 = 4.8


    def __init__(self, xi_0=0.1, xi_min=-0.35, xi_max=0.35):
        super(ControllerWithLinearization, self).__init__()
        self.xi = xi_0
        self.xi_min, self.xi_max = xi_min, xi_max


    def getControls(self, trajectory_point, x_cur, y_cur, dx_cur, dy_cur, theta, dt):

        # extracting of desired kinematic values
        x_r, dx_r, ddx_r, y_r, dy_r, ddy_r = trajectory_point.getPoint()

        # errors calculation
        err_x = x_r - x_cur
        err_dx = dx_r - dx_cur
        err_y = y_r - y_cur
        err_dy = dy_r - dy_cur

        # "simple" controller
        u1 = ddx_r + self.Kp_1 * err_x + self.Kd_1 * err_dx
        u2 = ddy_r + self.Kp_2 * err_y + self.Kd_2 * err_dy

        # converter's extra variable (xi)
        dot_xi = u1 * cos(theta) + u2 * sin(theta)
        self.xi += dot_xi * dt

        # limitation of xi
        if self.xi < self.xi_min:
            self.xi = self.xi_min
        if self.xi > self.xi_max:
            self.xi = self.xi_max

        # writing some of process info to file
        self.error_file.write("{0} {1} {2} {3} {4} {5} {6}\n".format(err_x, err_y, err_dx, err_dy, self.xi, u1, u2))

        # speeds calculation and return
        v_des = self.xi
        # zero value avoidance
        if self.xi != 0.0:
            omega_des = (- u1 * sin(theta) + u2 * cos(theta)) / self.xi
        else:
            omega_des = 0.0
        return v_des, omega_des
