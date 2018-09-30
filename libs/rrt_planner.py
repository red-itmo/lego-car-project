import random
import math
import numpy as np
import cv2
import time
import src.Auxilary as aux
random.seed()

class RRT_PLANNER:
	class Tree:
		class Vertex:
			def __init__(self, position, parent,child):
				self.position = position
				self.parent = parent
				self.child = child


		def __init__(self):
			self.edges = []
			self.vertexes = []


		def AddVertex(self, q_new):
			self.vertexes.append(q_new)


		def AddEdge(self, qnear, qnew):
			self.edges.append([qnear, qnew])


		def NearestNeighbor(self, new_vertex):
			min_dist = math.hypot(self.vertexes[0].position[0] - new_vertex.position[0], self.vertexes[0].position[1] - new_vertex.position[1])
			n_min = 0

			for n in range(len(self.vertexes)):
				candidate = math.hypot(self.vertexes[n].position[0] - new_vertex.position[0],self.vertexes[n].position[1] - new_vertex.position[1])
				if candidate < min_dist:
					min_dist = candidate
					n_min = n

			return n_min


		def PaintOnImg(self, img, obstacles):
			for vertex in self.vertexes:
				cv2.circle(img, (math.ceil(vertex.position[0]), math.ceil(vertex.position[1])), 2, (255,0,0))
			for edge in self.edges:
				cv2.line(img, (math.ceil(edge[0].position[0]), math.ceil(edge[0].position[1])),
							(math.ceil(edge[1].position[0]), math.ceil(edge[1].position[1])), (255,0,0))
			for obstacle in obstacles:
				for i in range(len(obstacle)-1):
					cv2.line(img, (obstacle[i][0], obstacle[i][1]), (obstacle[i+1][0], obstacle[i+1][1]), (0,255,0))
			return img


		def get_right_path(self, q_goal):
			path = []
			path.append(q_goal)
			next_point = q_goal.parent
			while next_point is not None:
				path.append(next_point)
				next_point = next_point.parent

			return path

		def draw_path(self, path, obstacles, img):
			for i in range(len(path) - 1):
				cv2.line(img, (math.ceil(path[i].position[0]), math.ceil(path[i].position[1])),
							(math.ceil(path[i+1].position[0]), math.ceil(path[i+1].position[1])), (0,0,255))
			for obstacle in obstacles:
				for i in range(len(obstacle) - 1):
					cv2.line(img, (obstacle[i][0],obstacle[i][1]), (obstacle[i+1][0],obstacle[i+1][1]), (0,255,0))
			return img

	def __init__(self):
		self.Tree = self.Tree()


	def get_path(self, img, start_pose, obstacles, goal_pose, n_of_iterations = 20, max_edge_px = 50):
		height, width = img.shape[:2]
		img_with_path = img
		q_root = self.Tree.Vertex(start_pose, None, None)
		self.Tree.AddVertex(q_root)
		for k in range(n_of_iterations):
			qrand_pos = [random.randint(0,width), random.randint(0,height)]
			qrand = self.Tree.Vertex(qrand_pos, None, None)
			n_qnear = self.Tree.NearestNeighbor(qrand)
			qnear = self.Tree.vertexes[n_qnear]
			qnew_pos = self.connect(qnear, qrand, obstacles, max_edge_px)
			qnew = self.Tree.Vertex(qnew_pos, None, None)
			self.Tree.AddVertex(qnew)
			self.Tree.AddEdge(qnear, qnew)
			qnear.child = qnew
			qnew.parent = qnear
			if (self.isInGoalCircle(qnew, goal_pose)):
				path = self.Tree.get_right_path(qnew)
				img_with_path = self.Tree.draw_path(path, obstacles, img_with_path)
				break

		return path


	def connect(self, vertex_1, vertex_2, obstacles, max_edge_px):
		candidate_dist = math.hypot(vertex_1.position[0] - vertex_2.position[0],vertex_1.position[1] - vertex_2.position[1])
		if(candidate_dist <= max_edge_px and not aux.CrossesObstacles(vertex_1.position, vertex_2.position, obstacles) and not aux.LiesBeneathObstacles(vertex_2.position, obstacles)):
			new_vertex = vertex_2.position
		else:
			new_vertex = [
							vertex_1.position[0] - max_edge_px*math.cos(math.atan2(vertex_1.position[1] - vertex_2.position[1], vertex_1.position[0] - vertex_2.position[0])),
							vertex_1.position[1] - max_edge_px*math.sin(math.atan2(vertex_1.position[1] - vertex_2.position[1], vertex_1.position[0] - vertex_2.position[0]))
						 ]

			new_vertex = [int(new_vertex[0]), int(new_vertex[1])]
			for obstacle in obstacles:
				InObstacle=True
				CrossesObstacle =True
				while InObstacle or CrossesObstacle:
					new_vertex = [
									vertex_1.position[0] - max_edge_px*math.cos(math.atan2(vertex_1.position[1] - vertex_2.position[1], vertex_1.position[0] - vertex_2.position[0])),
									vertex_1.position[1] - max_edge_px*math.sin(math.atan2(vertex_1.position[1] - vertex_2.position[1], vertex_1.position[0]  -vertex_2.position[0]))
								 ]
					CrossesObstacle = util.doIntersect(new_vertex, vertex_1.position, obstacle[0], obstacle[-1])
					InObstacle = False
					for point in obstacle:
						if(self.isInObstCircle(new_vertex, point)):
							InObstacle = True

					max_edge_px -= 1
		return new_vertex


	def onMouse(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONUP:
			self.clicked = True
		cv2.destroyWindow(self.win_name)
