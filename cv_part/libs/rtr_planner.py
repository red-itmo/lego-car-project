
import random
import math
import numpy as np
import cv2
import copy
import src.TrajectoryPlaners as TP
import src.Mapping as detect
import src.Auxilary as aux
random.seed()

# defining camera resolution and main diagonal of the floor on the camera
cam_res = (640, 480)
px_to_m = 3.60 / math.hypot(cam_res[0], cam_res[1])


class Robot:

	def __init__(self, p1, width, height):
		self.pos = [0, 0]
		self.width = width
		self.height = height
		self.centr_p1 = p1


	def get4pointsofrobot(self,q):
		r1 = math.sqrt(math.pow(self.width/4, 2) + math.pow(self.height/2, 2))
		r2 = math.sqrt(math.pow(self.width/4, 2) + math.pow(self.height/2, 2))
		r3 = math.sqrt(math.pow(self.width/4*3, 2) + math.pow(self.height/2, 2))
		r4 = math.sqrt(math.pow(self.width/4*3, 2) + math.pow(self.height/2, 2))


		angle1 = math.pi - math.acos(self.width/4/r1)
		angle2 = -angle1
		angle3 = math.acos(self.width*3/4/r3)
		angle4 = -angle3

		point1 = [q.x + r1*math.cos(angle1 + q.theta),
					q.y + r1*math.sin(angle1 + q.theta)]
		point2 = [q.x + r2*math.cos(angle2 + q.theta),
					q.y + r2*math.sin(angle2 + q.theta)]
		point4 = [q.x + r3*math.cos(angle3 + q.theta),
					q.y + r3*math.sin(angle3 + q.theta)]
		point3 = [q.x + r4*math.cos(angle4 + q.theta),
					q.y + r4*math.sin(angle4 + q.theta)]

		return [point1, point2, point3, point4]


	def ifCrash(self, q, obstacles, img, primitive, direction):
		points_of_robot = self.get4pointsofrobot(q)
		Crash = False
		turn_radius = math.sqrt(2)*self.height/2
		if(primitive == "translate" and direction == "forward"):
			points_of_robot[3] = [points_of_robot[3][0] + turn_radius*math.cos(q.theta), points_of_robot[3][1] + turn_radius*math.sin(q.theta)]
			points_of_robot[2] = [points_of_robot[2][0] + turn_radius*math.cos(q.theta), points_of_robot[2][1] + turn_radius*math.sin(q.theta)]
		if(primitive == "translate" and direction == "backward"):
			points_of_robot[0] = [points_of_robot[0][0] - turn_radius*math.cos(q.theta), points_of_robot[0][1] - turn_radius*math.sin(q.theta)]
			points_of_robot[1] = [points_of_robot[1][0] - turn_radius*math.cos(q.theta), points_of_robot[1][1] - turn_radius*math.sin(q.theta)]


		Crash = aux.CrossesObstacles(points_of_robot[0], points_of_robot[-1], obstacles)
		if(Crash):
			return True
		for i in range(len(points_of_robot)-1):
			Crash = aux.CrossesObstacles(points_of_robot[i], points_of_robot[i+1], obstacles)
			if(Crash):
				return True


		if(not aux.inRangeOfImg(points_of_robot, img)):
			return True
		return Crash

class Tree:

	class q:
		def __init__(self, position, theta):
			self.x = position[0]
			self.y = position[1]
			self.theta = theta
			self.parent = 1
			self.child = []


		def Translate(self, direction, cost):
			if(direction == "forward"):
				self.x += cost*math.cos(self.theta)
				self.y += cost*math.sin(self.theta)
			elif(direction == "backward"):
				self.x -= cost*math.cos(self.theta)
				self.y -= cost*math.sin(self.theta)
			else:
				print("Specify directionection by 'forward' or 'backward'")


		def Rotate(self, angle):
			self.theta += angle


		def __str__(self):
			return ("|" +str(self.x) + " " + str(self.y) + " " + str(self.theta) + "|" )


	class TCI:
		def __init__(self, q_begin, q_end):
			self.q_begin = q_begin
			self.q_end = q_end


	class RCI:
		def __init__(self, q_begin, q_end):
			self.q_begin = q_begin
			self.q_end = q_end


	def __init__(self, Robot, img):
		self.edges = []
		self.vertexes = []
		self.robot = Robot
		self.img = img


	def AddVertex(self, q_new):
		self.vertexes.append(copy.deepcopy(q_new))


	def AddEdge(self, qnear, qnew):
		self.edges.append([copy.deepcopy(qnear), copy.deepcopy(qnew)])


	def Init(self, q, obstacles):
		self.AddVertex(q)
		TCI = self.TCIExtend(q, "forward", obstacles)
		q.child.append(TCI.q_end)
		TCI.q_end.parent = q
		self.AddVertex(TCI.q_end)
		self.AddEdge(q, TCI)
		TCI = self.TCIExtend(q, "backward", obstacles)
		q.child.append(TCI.q_end)
		TCI.q_end.parent = q
		self.AddVertex(TCI.q_end)
		self.AddEdge(q, TCI)


	def Extend(self, q, turndirection, obstacles):
		RCI,collision = self.RCIExtend(q, turndirection, obstacles)
		q.child.append(RCI.q_end)
		RCI.q_end.parent = q
		self.AddVertex(RCI.q_end)
		self.AddEdge(q, RCI)
		TCI = self.TCIExtend(RCI.q_end, "forward", obstacles)
		q.child.append(TCI.q_end)
		TCI.q_end.parent = q
		self.AddVertex(TCI.q_end)
		self.AddEdge(RCI.q_end, TCI)
		TCI = self.TCIExtend(RCI.q_end, "backward", obstacles)
		q.child.append(TCI.q_end)
		TCI.q_end.parent = q
		self.AddVertex(TCI.q_end)
		self.AddEdge(RCI.q_end, TCI)

		return collision


	def TCIExtend(self, q, direction, obstacles):
		q_begin = copy.deepcopy(q)
		q_end = copy.deepcopy(q)
		while not self.robot.ifCrash(q_end, obstacles, self.img, "translate", direction):
			q_end.Translate(direction, 1)
		return self.TCI(q_begin, q_end)


	def RCIExtend(self, q, angle, obstacles):
		collision = False
		q_end = copy.deepcopy(q)
		q_begin = copy.deepcopy(q)
		rez_angle = 0
		while abs(rez_angle) < abs(angle) and not collision:
			rez_angle += math.copysign(1, angle)*0.1
			q_end.Rotate(math.copysign(1, angle)*0.1)
			collision = self.robot.ifCrash(q_end, obstacles, self.img, "rotate", "backward")


		if(collision):
			q_end.theta -= math.copysign(1, angle)*0.1

		return (self.RCI(q_begin, q_end), collision)


	def NearestNeighbor(self, new_vertex):
		min_dist = None
		min_q = None
		orient = None
		true_edge = None

		for edge in self.edges:
			if(edge[0].x == edge[1].q_end.x and edge[0].y == edge[1].q_end.y):
				candidate_q = [edge[1].q_end.x, edge[1].q_end.y]

			else:
				candidate_q = aux.ClosestPoint(edge[0], edge[1].q_end, self.q(new_vertex, 0))
			candidate_dist = math.hypot(new_vertex[0] - candidate_q[0], new_vertex[1] - candidate_q[1])
			if min_dist == None or candidate_dist < min_dist:
				true_edge = edge
				min_dist = candidate_dist
				min_q = candidate_q
				orient = edge[0].theta
		Q = self.q(min_q, orient)
		Q.parent = true_edge[0]
		return Q


	def draw_all(self, obstacles, color):
		img = self.img
		n = 0
		for vertex in self.vertexes:

			cv2.circle(img,(math.ceil(vertex.x), math.ceil(vertex.y)), 2, (0,0,255))
			if(n%50==0 or True):
				self.draw_robot(img, vertex, color)
		for edge in self.edges:
			cv2.line(img, (math.ceil(edge[0].x), math.ceil(edge[0].y)), (math.ceil(edge[1].q_end.x), math.ceil(edge[1].q_end.y)), (255,0,0))
		for obstacle in obstacles:
			for i in range(len(obstacle)-1):
				cv2.line(img, (obstacle[i][0],obstacle[i][1]), (obstacle[i+1][0],obstacle[i+1][1]), (0,255,0))

		win_name = "Win1"
		cv2.namedWindow(win_name)
		cv2.imshow(win_name,img)
		cv2.waitKey(0)


	def draw_robot(self, img, q, color):
		p_for_line = self.robot.get4pointsofrobot(q)
		cv2.circle(img,(int(q.x), int(q.y)), 5, (255,0,0), -1)
		for i in range(len(p_for_line)-1):
			cv2.line(img, (int(p_for_line[i][0]), int(p_for_line[i][1])), (int(p_for_line[i+1][0]), int(p_for_line[i+1][1])), color)
		cv2.line(img, (int(p_for_line[0][0]), int(p_for_line[0][1])), (int(p_for_line[-1][0]), int(p_for_line[-1][1])), color)
		return img


	def draw_path(self, obstacles, path, img, mode):
		for obstacle in obstacles:
			for i in range(len(obstacle)-1):
				cv2.line(img,(obstacle[i][0], obstacle[i][1]), (obstacle[i+1][0], obstacle[i+1][1]), (0,255,0))
		for n in range(len(path)-1):
			if(math.isnan(path[n].x) or math.isnan(path[n].y) or math.isnan(path[n].theta) or math.isnan(path[n+1].x) or math.isnan(path[n+1].y) or math.isnan(path[n+1].theta)):
				continue
			cv2.line(img, (math.ceil(path[n].x), math.ceil(path[n].y)), (math.ceil(path[n+1].x), math.ceil(path[n+1].y)), (255,0,0))
			font = cv2.FONT_HERSHEY_SIMPLEX
			if(mode):
				self.draw_robot(img, path[n], (0,0,255))
		self.draw_robot(img, path[-1], (0,0,255))
		return img


class RTR_PLANNER:

	def __init__(self, Robot, img):
		self.win_name = "Win1"
		self.Tree_end = Tree(Robot, img)
		self.Tree_root = Tree(Robot, img)
		self.dims = [img.shape[:2][1],img.shape[:2][0]]


	def construct(self, q_init, q_end, obstacles, max_expand_dist = 50, n_of_iterations = 50):
		self.Tree_root.Init(q_init, obstacles)
		self.Tree_end.Init(q_end, obstacles)
		for k in range(n_of_iterations):
			P_g_init = self.RandomPos(self.dims)
			P_g_end = self.RandomPos(self.dims)
			q_near_init = self.Tree_root.NearestNeighbor(P_g_init)
			q_near_end = self.Tree_end.NearestNeighbor(P_g_end)
			turndirection_init = self.MinTurndirection(q_near_init, P_g_init)
			turndirection_end = self.MinTurndirection(q_near_end, P_g_end)
			collision_init = self.Tree_root.Extend(q_near_init, turndirection_init, obstacles)
			collision_end = self.Tree_end.Extend(q_near_end, turndirection_end, obstacles)
			if collision_init:
				self.Tree_root.Extend(q_near_init, -math.copysign(1, turndirection_init)*(2*math.pi - abs(turndirection_init)), obstacles)
			if collision_end:
				self.Tree_end.Extend(q_near_end, -math.copysign(1, turndirection_end)*(2*math.pi - abs(turndirection_end)), obstacles)
			n,goal_achieved = self.checkGoal(self.Tree_root.edges[-1], self.Tree_end.edges, obstacles)

			if(goal_achieved):
				path = self.getpath(q_init, self.Tree_root.edges[-1], q_end, self.Tree_end.edges[n])
				print("Number of iterations: " + str(k))
				return path

		return None


	def checkGoal(self, edge_init, edges_end, obstacles):
		for n in range(len(edges_end)):
			if(type(edges_end[n][1]) is Tree.RCI or type(edge_init[1]) is Tree.RCI):
				continue
			if(aux.doIntersect([edge_init[1].q_begin.x, edge_init[1].q_begin.y], [edge_init[1].q_end.x, edge_init[1].q_end.y],
									[edges_end[n][1].q_begin.x, edges_end[n][1].q_begin.y], [edges_end[n][1].q_end.x, edges_end[n][1].q_end.y])):

				intersection = aux.line_intersection([[edge_init[0].x, edge_init[0].y], [edge_init[1].q_end.x, edge_init[1].q_end.y]],
													[[edges_end[n][0].x, edges_end[n][0].y], [edges_end[n][1].q_end.x, edges_end[n][1].q_end.y]])
				q_inter = Tree.q(intersection, edge_init[1].q_end.theta)
				angle = self.MinTurndirection(q_inter, [edges_end[n][1].q_end.x, edges_end[n][1].q_end.y])
				_,collision = self.Tree_root.RCIExtend(q_inter, angle, obstacles)
				if(not collision):
					return n,True

		return None,False


	def getpath(self, q_init, edge_init, q_end, edge_end):
		path1 = []
		q_step = edge_init[0]
		while(q_step is not None):
			path1.append(q_step)
			q_step = q_step.parent

		path1 = path1[::-1]
		path2 = []
		q_step = edge_end[0]
		while(q_step is not None):
			path2.append(q_step)
			q_step = q_step.parent

		intersection = aux.line_intersection([[edge_init[0].x, edge_init[0].y], [edge_init[1].q_end.x, edge_init[1].q_end.y]],
												[[edge_end[0].x, edge_end[0].y], [edge_end[1].q_end.x, edge_end[1].q_end.y]])
		path1.append(Tree.q(intersection, path1[-1].theta))
		path1.extend(path2)

		return path1


	def find(self, q_begin, tree):
		for edge in tree.edges:
			if(type(edge[1]) is Tree.RCI):
				continue
			if(edge[1].q_end == q_begin and edge[1].q_begin != q_begin):
				return edge[1]


	def correct_angles_in_path(self, path):
		correct_path = []
		for n in range(len(path)-1):
			if(n==0):
				correct_path.append(path[0])
				continue
			correct_path.append(Tree.q([path[n].x, path[n].y], path[n].theta + self.MinTurndirection(path[n], [path[n+1].x, path[n+1].y])))
		correct_path.append(Tree.q([path[-1].x, path[-1].y], path[-1].theta))
		return correct_path


	def MinTurndirection(self, q_nearest, random_point):
		x = random_point[0] - q_nearest.x
		y = random_point[1] - q_nearest.y

		dx = math.cos(q_nearest.theta)
		dy = math.sin(q_nearest.theta)
		angle = math.atan2(x*dy - y*dx, x*dx + y*dy)

		return -angle


	def RandomPos(self,dims):
		return [random.randint(0, dims[0]), random.randint(0, dims[1])]


	def print_path(self,path):
		print("~~~~~~~~~~~~~~~")
		for q in path:
			print(q)
		print("~~~~~~~~~~~~~~\n")


	def generate_paths(self,true_path, p_type, n_of_samples,reverse=False):
		paths = []
		for i in range(n_of_samples-1):
			path = TP.TTSPlaner().getTrajectory(TP.Pose(true_path[0].x, true_path[0].y, true_path[0].theta),
													TP.Pose(true_path[-1].x, true_path[-1].y, true_path[-1].theta),
														0.2, 0.05, reverse)
			paths.append(path)

		paths.append(TP.eeS_Planer().getTrajectory(TP.Pose(true_path[0].x, true_path[0].y, true_path[0].theta),
														TP.Pose(true_path[-1].x, true_path[-1].y, true_path[-1].theta),
															0.2, 0.05, reverse))
		paths = self.into_q(paths)
		return paths


	def into_q(self,paths):
		new_path = []
		for path in paths:
			temp = [path[0], path[1]]
			temp_path = []
			for q in path[2][1:]:
				temp_path.append(self.Tree_end.q([q[0], q[1]], q[2]))
			temp.append(temp_path)
			new_path.append(temp)
		return new_path


	def get_path_len(self, path):
		length = 0
		for n in range(len(path)-1):
			length+=math.hypot(path[n].x - path[n+1].x, path[n].y - path[n+1].y)
		return length


	def transform_path(self, true_path, p_type, obstacles, frame, robot, n_of_samples, reverse = False):
		paths = self.generate_paths(true_path, p_type, n_of_samples, reverse)
		min_path = None
		min_path_dist = 10000000
		exist = False
		for path in paths:
			safe = self.check_for_safety(path[2], obstacles, frame, robot)
			if(safe):
				cand_dist = np.asscalar(path[1])
				if(cand_dist < min_path_dist):
					mim_path_dist = cand_dist
					min_path = path
					exist = True

		if(exist):
			return min_path

		else:
			if(self.get_path_len(true_path) < 0.1):
				return False

			mid_paths = self.find_middle_of_path(true_path)

			path_lo = mid_paths[0]
			path_hi = mid_paths[1]
			cor_path_lo = self.transform_path(path_lo, p_type, obstacles, frame, robot, n_of_samples)
			cor_path_hi = self.transform_path(path_hi, p_type, obstacles, frame, robot, n_of_samples)
			if(cor_path_lo and cor_path_hi):
				return self.concat_paths(cor_path_lo, cor_path_hi)
			else:
				return False


	def concat_paths(self, cor_path_lo, cor_path_hi):
		if(type(cor_path_hi[1]) is float):
			a = cor_path_hi[1]
		else:
			a = np.asscalar(cor_path_hi[1])
		if(type(cor_path_lo[1]) is float):
			b = cor_path_lo[1]
		else:
			b = np.asscalar(cor_path_lo[1])
		path = cor_path_lo[2] + cor_path_hi[2]
		cor_path = [cor_path_lo[0] + cor_path_hi[0], a+b, path]
		return cor_path


	def check_for_safety(self,x_y_z, obstacles, frame, robot):
		for q in x_y_z:
			candidate = self.Tree_end.q([q.x/px_to_m, q.y/px_to_m], q.theta)
			if(robot.ifCrash(candidate, obstacles, frame, None, None)):
				return False
		return True


	def path_into_m(self, path):
		new_path = []
		for q in path:
			new_path.append(self.Tree_end.q([q.x*px_to_m, q.y*px_to_m], q.theta))
		return new_path


	def path_to_px(self, path):
		new_path = []
		for q in path:
			new_path.append(self.Tree_end.q([q.x/px_to_m, q.y/px_to_m], q.theta))
		return new_path


	def find_middle_of_path(self, path):
		length = 0
		path_lo = []
		path_ho = []
		for n in range(len(path)-1):
			length += math.hypot(path[n].x - path[n+1].x, path[n].y - path[n+1].y)
		mid = length/2
		len_f = 0
		for n in range(len(path)-1):
			len_f+=math.hypot(path[n].x - path[n+1].x, path[n].y - path[n+1].y)
			if(len_f >= mid):
				if(len_f == mid):
					mid_x_y = [path[n+1].x, path[n+1].y]
					mid_theta = path[n+1].theta
				else:
					dist = (-len_f + math.hypot(path[n].x - path[n+1].x, path[n].y - path[n+1].y) + mid)
					angle = math.atan2(-path[n].y + path[n+1].y, -path[n].x + path[n+1].x)
					mid_x_y = [path[n].x + dist*math.cos(angle),path[n].y + dist*math.sin(angle)]
					mid_theta = path[n+1].theta
				mid_q = self.Tree_end.q(mid_x_y, mid_theta)
				path_lo = path[0:n+1]
				path_lo.append(mid_q)
				path_hi = []
				path_hi.append(mid_q)
				path_hi.extend(path[n+1:])
				return path_lo, path_hi
