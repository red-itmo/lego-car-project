#!/usr/bin/env python3
from math import radians
from ev3dev.ev3 import Sound, Button, GyroSensor, UltrasonicSensor, LargeMotor, OUTPUT_A, OUTPUT_B

from libs.Clock import Clock
from libs.PID import PID

from libs.Localization import Localization
from libs.VelocityController import VelocityController
from libs.PathController import PathController

from libs.TrajectoryControllers import ControllerWithLinearization


class RobotState:
    """
        !!! Not used but can be used!
        !!! needs a separate thread
    """
    def __init__(self, robot):
        self.x, self.y = 0, 0
        self.dx, self.dy = 0, 0
        self.theta, self.omega = 0, 0

        self.__gyro = robot.getDevice('GYRO')
        self.__motor_rear, self.__motor_steer = robot.getDevice('MOTORS')

        self.__localization = Localization(self)

    def update(self):
        clock = Clock()
        while True:
            t, dt = clock.getTandDT()
            self.theta, self.omega = [-x for x in self.__gyro.rate_and_angle]
            self.x, self.y, self.dx, self.dy = self.__localization.getData(radians(self.theta), radians(self.__motor_rear.speed), dt)

    def getState(self):
        return self.x, self.y, self.dx, self.dy, self.theta, self.omega


class LegoCar:

    # Parameters of the car
    HARD_MAX_PHI = 0.61
    SOFT_MAX_PHI = 0.3
    R = 0.0432 / 2          # wheel radius
    L = 0.165            # wheelbase

    def __init__(self, init_pose, soc=None, port_motor_rear=OUTPUT_A, port_motor_steer=OUTPUT_B, port_sensor_gyro='in1'):

        # initialization of devices
        self.__button_halt = Button()
        self.__motor_rear = LargeMotor(port_motor_rear)
        self.__motor_steer = LargeMotor(port_motor_steer)
        self.__sensor_gyro = GyroSensor(port_sensor_gyro)

        self.__velocity_controller = VelocityController(self, 0, 0, adaptation=False)
        self.s = soc
        # NOTE: possible using other controllers. Change only here!
        self.__trajectory_controller = ControllerWithLinearization()
        self.__path_controller = PathController()

        self.__localization = Localization(self, init_pose)
        self.__robot_state = [0, 0, 0, 0, 0, 0]    # x,y, dx,dy, theta,omega

        self.reset()
        Sound.speak('Ready').wait()

    def reset(self):
        # reset encoders of motors
        self.__motor_rear.reset()
        self.__motor_steer.reset()
        # reset gyro sensor. The robot needs to be perfectly still!
        self.__sensor_gyro.mode = self.__sensor_gyro.modes[1]
        self.__sensor_gyro.mode = self.__sensor_gyro.modes[0]

    def getDevice(self, what):
        if what in ['MOTORS']:
            return self.__motor_rear, self.__motor_steer
        elif what in ['GYRO']:
            return self.__sensor_gyro

    def getState(self):
        return self.__robot_state

    def turnWheel(self, phi_desired):
        """ Turns front wheels on the desired angle 'phi_desired' """
        pid_phi = PID(100.0, 500.0, 5.0, 100, -100)
        t = 0
        clock = Clock()
        while t < 1:  # FIXME: seems that need steady state condition
            t, dt = clock.getTandDT()

            phi_current = self.__motor_steer.position
            error_phi = radians(phi_desired - phi_current)

            u_phi = pid_phi.getControl(error_phi, dt)
            self.__motor_steer.run_direct(duty_cycle_sp=u_phi)
        Sound.speak('Wheels were turned!')

    def velocity_move(self, vel_linear, vel_angular, time):
        # initialization for current mode
        self.__velocity_controller.setTargetVelocities(vel_linear, vel_angular)
        clock = Clock()
        fh = open("vel.txt", "w")
        while clock.getCurrentTime() <= time:
            try:
                t, dt = clock.getTandDT()

                theta, omega = [-x for x in self.__sensor_gyro.rate_and_angle]  # returns ANGLE AND RATE
                x, y, dx, dy, v_r = self.__localization.getData(radians(theta), radians(self.__motor_rear.speed), dt)
                self.__robot_state = [x, y, dx, dy, theta, omega]  # update state

                u_v, u_phi = self.__velocity_controller.getControls(radians(self.__motor_rear.speed),
                                                                    radians(self.__motor_steer.position),
                                                                    radians(omega), dt)

                fh.write("%f %f %f \n" % (t, v_r, radians(omega)))
                self.__motor_rear.run_direct(duty_cycle_sp=u_v)
                self.__motor_steer.run_direct(duty_cycle_sp=u_phi)

                if self.__button_halt.enter:
                    break
            except KeyboardInterrupt:
                break
        # off motors
        fh.close()
        self.__motor_rear.duty_cycle_sp = 0
        self.__motor_steer.duty_cycle_sp = 0
        # raise SystemExit

    def trajectory_move(self, trajectory):
        clock = Clock()
        fh = open("test.txt", "w")
        while not trajectory.is_end:

            try:
                t, dt = clock.getTandDT()

                theta, omega = [-x for x in self.__sensor_gyro.rate_and_angle]  # !!! returns ANGLE AND RATE :)
                x, y, dx, dy, v_r, beta = self.__localization.getData(radians(theta), radians(self.__motor_rear.speed), dt)

                self.__robot_state = [x, y, dx, dy, theta, omega]  # update state

                point = trajectory.getCoordinatesTime(t)
                v_des, omega_des = self.__trajectory_controller.getControls(point, x, y, dx, dy, beta, dt)
                self.__velocity_controller.setTargetVelocities(v_des, omega_des)

                u_v, u_phi = self.__velocity_controller.getControls(radians(self.__motor_rear.speed),
                                                                    radians(self.__motor_steer.position),
                                                                    radians(omega), dt)
                fh.write("%f %f %f %f %f %f %f %f %f %f %f\n" % (t, x, y, point.x, point.y, v_r, v_des, theta, radians(omega), omega_des, self.__motor_steer.position))
                self.__motor_rear.run_direct(duty_cycle_sp=u_v)
                self.__motor_steer.run_direct(duty_cycle_sp=u_phi)
                #print(trajectory.is_end)
                if self.__button_halt.enter:
                    break
            except KeyboardInterrupt:
                break
        # off motors
        fh.close()
        self.__motor_rear.duty_cycle_sp = 0
        self.__motor_steer.duty_cycle_sp = 0
        # raise SystemExit

    def path_move(self, trajectory, v_des = 0.2):
        fh = open("test.txt", "w")
        clock = Clock()
        while not trajectory.is_end:
            try:
                t, dt = clock.getTandDT()

                theta, omega = [-x for x in self.__sensor_gyro.rate_and_angle]  # !!! returns ANGLE AND RATE :)
                x, y, dx, dy, v_r = self.__localization.getData(radians(theta), radians(self.__motor_rear.speed), dt)
                self.__robot_state = [x, y, dx, dy, theta, omega]  # update state

                x_ref, y_ref, kappa, t_hat = trajectory.getCoordinatesDistance(x, y)
                omega_des = self.__path_controller.getControls(v_r, x, y, x_ref, y_ref, radians(theta), kappa, t_hat)
                self.__velocity_controller.setTargetVelocities(v_des, omega_des)

                u_v, u_phi = self.__velocity_controller.getControls(radians(self.__motor_rear.speed),
                                                                    radians(self.__motor_steer.position),
                                                                    radians(omega), dt)

                fh.write("%f %f %f %f %f\n" %(t, x, y, x_ref, y_ref))
                self.__motor_rear.run_direct(duty_cycle_sp=u_v)
                self.__motor_steer.run_direct(duty_cycle_sp=u_phi)

                if self.__button_halt.enter:
                    break
            except KeyboardInterrupt:
                break
        # off motors
        self.__motor_rear.duty_cycle_sp = 0
        self.__motor_steer.duty_cycle_sp = 0
        # raise SystemExit
