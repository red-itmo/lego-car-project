#!/usr/bin/env python3
from math import pi, sin, cos, sqrt, atan2


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
        else
            omega_des = 0.0
        return v_des, omega_des


class ControllerWithCoordinateTransform(TrajectoryController):

    # controller coefficients
    k_1 = 1.0
    k_2 = 5.0
    k_3 = 5.0


    def __init__(self, reverse_gear=False):
        super(ControllerWithCoordinateTransform, self).__init__()
        self.reverse_gear = reverse_gear


    def getControls(self, trajectory_point, x_cur, y_cur, dx_cur, dy_cur, theta, dt):

        # extracting of desired kinematic values
        x_r, dx_r, ddx_r, y_r, dy_r, ddy_r = trajectory_point.getPoint()

        # calculating of some other desired kinematic values
        v_ref = sqrt(dx_r**2 + dy_r**2) # desired linear velocity
        if v_ref != 0.0:
            tau = [dx_r / v_ref, dy_r / v_ref]
            a_t = (dx_r * ddx_r + dy_r * ddy_r) / v_ref
            a_t = [a_t * tau[0], a_t * tau[1]]  # desired rotational (tangential) acceleration
            a_c = [ddx_r - a_t[0], ddy_r - a_t[1]]  # desired centripetal acceleration
            omega_ref = 0.0 # desired angular velocity
            if dx_r != 0.0:
                omega_ref += 0.5 * a_c[1] / dx_r
            if dy_r != 0.0:
                omega_ref -= 0.5 * a_c[0] / dy_r
        else:
            omega_ref = 0.0
        theta_ref = atan2(y_r, x_r) #TODO#1 problems with angle's boundaries may occur here

        # turn on reverse gear if needed
        if self.reverse_gear:
            v_ref = -v_ref
            theta_ref += pi #TODO#2 problems with angle's boundaries may occur here

        # errors (in robot's frame) calculation
        x_e = cos(theta) * (x_r - x_cur) + sin(theta) * (y_r - y_cur)
        y_e = -sin(theta) * (x_r - x_cur) + cos(theta) * (y_r - y_cur)
        theta_e = theta_ref - theta

        # writing some of process info to file
        print("{0} {1} {2}\n".format(x_e, y_e, theta))

        # speeds calculation and return
        v_des = v_ref * cos(theta_e) + self.k_1 * x_e
        omega_des = omega_ref + v_ref * (self.k_2 * y_e + self.k_3 * sin(theta_e))
        return v_des, omega_des
