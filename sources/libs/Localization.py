from math import sin, cos, pi
import libs.Client as Client
import copy

class Localization:
	"""
        Using Odometry (linear speed of rear motor) + gyro angle
        compute position and velocity in cartesian space.
    """

	def __init__(self, car, init_pose):
		self.__car = car
		self.__x = 0
		self.x_car = 0
		self.y_car = 0
		self.x_init = init_pose[0]
		self.y_init = init_pose[1]
		self.__y = 0
		self.alpha = init_pose[2]
		#print("X_init {}, Y_init {}, Angle_init {}".format(self.x_init, self.y_init, self.alpha))

	def getData(self, theta, motor_speed, dt):
		alpha = copy.deepcopy(self.alpha)
		# robot's velocity
		v = motor_speed * self.__car.R
		# robot's linear velocities
		theta = theta + alpha
		vx = v * cos(theta)
		vy = v * sin(theta)

		self.x_car += vx * dt
		self.y_car += vy * dt

		if self.__car.s is None:
			# robot's coordinates
			self.__x += vx * dt
			self.__y += vy * dt
		else:
			while True:
				coord = self.__car.s.get_data()
				print(coord)
				if isinstance(coord[0],float):
					break
			print('----------------------------------')
			print(coord)

			self.__x = self.x_car * 0 + 1 * (-sin(alpha) * coord[1] + cos(alpha) * coord[0] + sin(alpha) * self.y_init - cos(alpha) * self.x_init)
			self.__y = self.y_car * 0 + 1 * (-cos(alpha) * coord[1] - sin(alpha) * coord[0] + cos(alpha) * self.y_init + sin(alpha) * self.x_init)
			#print("x: {} y: {} x_car: {} y_car: {}".format(self.__x, self.__y, self.x_car, self.y_car))

		return self.__x, self.__y, vx, vy, v, theta
