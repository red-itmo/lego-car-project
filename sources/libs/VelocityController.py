#!/usr/bin/env python3
from math import atan, copysign
from libs.Identificator import Identificator
from libs.PID import PID

R = 0.0432 / 2

class VelocityController:

    def __init__(self, robot, v_desired, omega_desired, adaptation=False):
        self.adaptation = adaptation
        self.identificator = Identificator()
        self.robot = robot
        self.v_desired = v_desired
        self.omega_desired = omega_desired
        self.pid_v = PID(10.0 / R , 90.0 / R, 0.05 / R, 100, -100)
        self.pid_phi = PID(100.0, 500.0, 5.0, 100, -100)


    def getControls(self, rear_motor_speed, phi, omega, dt):

        # forward motion control
        v_current = rear_motor_speed * self.robot.R
        error_v = self.v_desired - v_current
        u_v = self.pid_v.getControl(error_v, dt)

        if self.adaptation == True:
            theta = self.identificator.update(v_current, omega, phi)


        # steering control
        if v_current != 0:
            if not self.adaptation:
                phi_desired = atan(self.robot.L * self.omega_desired / v_current)
            else:
                phi_desired = 1 / theta[0][0] * (self.robot.L * self.omega_desired / v_current - theta[1][0])
            if not -self.robot.SOFT_MAX_PHI < phi_desired < self.robot.SOFT_MAX_PHI:
                phi_desired = self.robot.SOFT_MAX_PHI * copysign(1, phi_desired)
        else:
            phi_desired = 0.0

        error_phi = phi_desired - phi
        u_phi = self.pid_phi.getControl(error_phi, dt)

        # MAYBE must be deleted
        if abs(self.v_desired) < 0.05:
            u_v, u_phi = 0, 0

        return u_v, u_phi


    def setTargetVelocities(self, v_desired, omega_desired):
        self.v_desired = v_desired
        self.omega_desired = omega_desired
