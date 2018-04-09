#!/usr/bin/env python3
"""
    Module launching the lego-car
"""
from libs.Robots import LegoCar
from libs.TrajectoryStuff import Point, StraightLine, CircleLine, ClothoidLine, Pose
import libs.Client as Client
import libs.Localization as Loc
from math import cos, sin, pi
from numpy import array, asscalar


class Transformer:
	def __init__(self, initP):
		self.init_Pose = Pose( initP[0], initP[1], initP[2] )

	def transformPoint(self, point):
		theta = self.init_Pose.angle
		x_init = self.init_Pose.point.x
		y_init = self.init_Pose.point.y
		frame_x = point.x
		frame_y = point.y

		x = -sin(theta) * frame_y + cos(theta) * frame_x + sin(theta) * y_init - cos(theta) * x_init
		y = -cos(theta) * frame_y - sin(theta) * frame_x + cos(theta) * y_init + sin(theta) * x_init
		return Point(x, y)

	def transformPose(self, pose):
		theta = self.init_Pose.angle
		angle = -pose.angle
		x_init = self.init_Pose.point.x
		y_init = self.init_Pose.point.y
		frame_x = pose.point.x
		frame_y = pose.point.y

		x = -sin(theta) * frame_y + cos(theta) * frame_x + sin(theta) * y_init - cos(theta) * x_init
		y = -cos(theta) * frame_y - sin(theta) * frame_x + cos(theta) * y_init + sin(theta) * x_init
		return Pose(x, y, angle)


# TODO: coordinates transform!!!
def decode_trajectory(path, tr):
	full_trajectory = []
	for traj in path:
		print(traj)
		if traj[0] == 'ClothoidLine':
			# ['ClothoidLine', array([[0.225, 0.225, 0.]]), array([1.]), array([53.00920797]), array([0.02321948]), 'in', array([0.2])]
			f_pose = Pose(traj[1][0,0], traj[1][0, 1], traj[1][0, 2])
			pose = tr.transformPose(f_pose)
			t = ClothoidLine(pose, traj[2][0], -traj[3][0], traj[4][0], traj[6][0], type=traj[5])
			full_trajectory.append(t)
		elif traj[0] == 'CircleLine':
			# TODO: question about descriptor
			# ['CircleLine', array([[1.08397562, 1.36841653, 2.49348083]]), array([[0.63640656, 0.77735505, 2.49348083]]), array([[0.38751219, 1.62260234, 1.22085104]]), 'cw', array([0.2])]
			f_centerPoint = Point( traj[1][0, 0], traj[1][0, 1] )
			f_startPoint = Point( traj[2][0, 0], traj[2][0, 1] )
			f_endPoint = Point( traj[3][0, 0], traj[3][0, 1] )
			vel = asscalar( traj[5] )
			dir = traj[4]

			centerPoint = tr.transformPoint( f_centerPoint )
			startPoint = tr.transformPoint(f_startPoint)
			endPoint = tr.transformPoint(f_endPoint)

			if dir == 'cw':
				t = CircleLine(centerPoint, startPoint, endPoint, 'ccw', v=vel)
			else:
				t = CircleLine(centerPoint, startPoint, endPoint, 'cw', v=vel)
			full_trajectory.append(t)

		elif traj[0] == 'StraightLine':
			# ['StraightLine', array([[0.21043554, 1.02747685, 2.6]]), array([[0.49105373, 0.8586579, 2.6]]), 0.3274849713622936, 0.2]
			f_startPoint = Point( traj[1][0, 0], traj[1][0, 1] )
			f_endPoint = Point( traj[2][0, 0], traj[2][0, 1] )
			vel = traj[4]

			startPoint = tr.transformPoint( f_startPoint )
			endPoint = tr.transformPoint(f_endPoint)

			t = StraightLine(startPoint, endPoint, v=vel)
			full_trajectory.append(t)
	return full_trajectory


if __name__ == '__main__':
	# create car instance

	sock = Client.Client(3000)

	while True:
		data = sock.get_data()
		if data:
			break

	# data = [['ClothoidLine', array([[ 0.513     ,  1.701     , -1.64210379]]), array([-1.]), array([ 9.26478451]), array([ 0.14571305]), 'in', array([ 0.2])], ['ClothoidLine', array([[ 0.51922143,  1.84965295, -1.54374748]]), array([-1.]), array([-9.26478451]), array([ 0.14571305]), 'out', array([ 0.2])], ['ClothoidLine', array([[ 0.50497124,  1.99775203, -1.44539118]]), array([ 1.]), array([ 1.21588237]), array([ 1.11030477]), 'in', array([ 0.2])], ['ClothoidLine', array([[ 0.9063962 ,  0.98787121, -0.69593545]]), array([ 1.]), array([-1.21588237]), array([ 1.11030477]), 'out', array([ 0.2])], ['StraightLine', array([[ 2.19719456,  0.78467114,  0.05352027]]), array([[ 1.97159728,  0.77258557,  0.05352027]]), 0.22592076696823815, 0.2]]


	init_pose = data[0]
	print("INIT POSE: ",init_pose)
	#init_pose = [0, 0, 0]
	path = data[1]
	#end = data[1]
	#print("END POSE: ", end)
	car = LegoCar(init_pose, sock)


	convert_coordinates = Transformer(init_pose)



	# print("Path: ", path)


	# init_pose = end_point[0]
	# end_point = end_point[1]




	# car = LegoCar([0, 0, 0], None)

	# startPoint = transform(f_startPoint, f_initPose)
	# centerPoint = transform(f_centerPoint, f_initPose)
	# endPoint = transform(f_endPoint, f_initPose)

	# sock)

	# print(d)

	# trajectory tracking
	# move with desired velocities
	# car.velocity_move(vel_linear= 0.2, vel_angular=0.0, time=5.0)
	# t = CircleLine(centerPoint, startPoint, endPoint, dir, v=vel)
	# TODO: FIXME
	# t = StraightLine( startPoint, endPoint, vel )
	# t = StraightLine(Point(0, 0), Point(2, 0), 0.1)
	#car.trajectory_move(trajectory=t)
	# t1 = ClothoidLine(Pose(0, 0, 0), 1, 0.349902, 3.793262, 0.2, type="in")
	# t2 = ClothoidLine(Pose(2, 2, 2.5173395), 1, -0.349902, 3.793262, 0.2, type="out")
	# trajectory = [t1, t2]
	# #trajectory = [t]
	#
	#
	#st = convert_coordinates.transformPoint( Point( init_pose[0], init_pose[1] ) )
	#en = convert_coordinates.transformPoint( Point( end[0], end[1] ) )

	#st = Pose(  )

	trajectory = decode_trajectory( path, convert_coordinates )
	#trajectory = [ ClothoidLine( st, en, 0.2 ) ]
	for t in trajectory:
         print("New trajectory")
         car.trajectory_move(trajectory=t)
