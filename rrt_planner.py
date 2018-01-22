import random
import math
import numpy as np
import cv2
import time
random.seed()

class RRT_PLANNER:

	class Tree:

		class Vertex:

			def __init__(self,position,parent,child):
				self.position = position
				self.parent = parent
				self.child = child


		def __init__(self):
			self.edges = []
			self.vertexes = []

		def AddVertex(self,q_new):
			self.vertexes.append(q_new)

		def AddEdge(self,qnear,qnew):
			self.edges.append([qnear,qnew])

		def NearestNeighbor(self,new_vertex):
			min_dist = math.hypot(self.vertexes[0].position[0]-new_vertex.position[0],self.vertexes[0].position[1]-new_vertex.position[1]) 
			n_min = 0

			for n in range(len(self.vertexes)):
				candidate = math.hypot(self.vertexes[n].position[0]-new_vertex.position[0],self.vertexes[n].position[1]-new_vertex.position[1]) 
				if candidate < min_dist:
					min_dist = candidate
					n_min = n
			
			return n_min

		def PaintOnImg(self,img,obstacles):
			for vertex in self.vertexes:
				#print(vertex.position)
				cv2.circle(img,(math.ceil(vertex.position[0]),math.ceil(vertex.position[1])),2,(255,0,0))
			for edge in self.edges:
				cv2.line(img,(math.ceil(edge[0].position[0]),math.ceil(edge[0].position[1])),(math.ceil(edge[1].position[0]),math.ceil(edge[1].position[1])),(255,0,0))
			for obstacle in obstacles:
				for i in range(len(obstacle)-1):
					cv2.line(img,(obstacle[i][0],obstacle[i][1]),(obstacle[i+1][0],obstacle[i+1][1]),(0,255,0))
			return img

		def get_right_path(self,q_goal):
			path = []
			path.append(q_goal)
			next_point = q_goal.parent
			while next_point is not None:
				path.append(next_point)
				next_point = next_point.parent

			return path
				
		def draw_path(self,path,obstacles,img):
			for i in range(len(path)-1):
				cv2.line(img,(math.ceil(path[i].position[0]),math.ceil(path[i].position[1])),
							(math.ceil(path[i+1].position[0]),math.ceil(path[i+1].position[1])),(0,0,255))
			for obstacle in obstacles:
				for i in range(len(obstacle)-1):
					cv2.line(img,(obstacle[i][0],obstacle[i][1]),(obstacle[i+1][0],obstacle[i+1][1]),(0,255,0))
			return img

	def __init__(self):
		self.win_name = "Win1"
		cv2.namedWindow(self.win_name)
		#cv2.setMouseCallback(self.win_name,self.onMouse)
		self.Tree = self.Tree()

	def get_path(self,img,start_pose,obstacles,goal_pose,n_of_iterations = 20,max_edge_px = 50):
		height, width = img.shape[:2]
		img_with_path = img
		#cv2.imshow(self.win_name,img_with_path)
		#rint(img_with_path)
		#cv2.waitKey(0)
		q_root = self.Tree.Vertex(start_pose,None,None)
		self.Tree.AddVertex(q_root)
		cv2.circle(img_with_path,(start_pose[0],start_pose[1]),5,(0,255,0),-1)
		cv2.circle(img_with_path,(goal_pose[0],goal_pose[1]),5,(0,0,255),-1)	
		cv2.imshow(self.win_name,img_with_path)
		cv2.waitKey(0)
		for k in range(n_of_iterations):
			qrand_pos = [random.randint(0,width),random.randint(0,height)]
			qrand = self.Tree.Vertex(qrand_pos,None,None)
			n_qnear = self.Tree.NearestNeighbor(qrand)
			qnear = self.Tree.vertexes[n_qnear]
			qnew_pos = self.connect(qnear,qrand,obstacles,max_edge_px)
			qnew = self.Tree.Vertex(qnew_pos,None,None)
			self.Tree.AddVertex(qnew)
			self.Tree.AddEdge(qnear,qnew)
			qnear.child = qnew
			qnew.parent = qnear 
			#print(self.Tree.vertexes)
			#print(self.Tree.edges)
			cv2.imshow(self.win_name,self.Tree.PaintOnImg(img_with_path,obstacles))
			cv2.waitKey(1)
			if (self.isInGoalCircle(qnew,goal_pose)):
				path = self.Tree.get_right_path(qnew)
				img_with_path = self.Tree.draw_path(path,obstacles,img_with_path)
				break
				
		print("N of iter: ",k)
		print("Success")
		cv2.imshow(self.win_name,img_with_path)
		cv2.imwrite('rrt_planner_through_narrow_corridor.png', img_with_path)
		cv2.waitKey(0)
		return path

	def isInGoalCircle(self,point,goal):
		d = math.sqrt(math.pow((point.position[0]-goal[0]),2)+math.pow((point.position[1]-goal[1]),2))
		return d<20

	def isInObstCircle(self,point,obs_point):
		d = math.sqrt(math.pow((point[0]-obs_point[0]),2)+math.pow((point[1]-obs_point[1]),2))
		return d<40

	def onSegment(self,p,q,r):
		if(q[0]<=max(p[0],r[0]) and q[0]>=min(p[0],r[0]) 
				and q[1]<=max(p[1],r[1]) and q[1]>=min(p[1],r[1])):
			return True
		return False

	def orientation(self,p,q,r):
		val = (q[1]-p[1])*(r[0]-q[0])-(q[0]-p[0])*(r[1]-q[1])

		if(val==0):
			return 0

		return 1 if val>0 else 2

	def doIntersect(self,p_1_begin,p_1_end,p_2_begin,p_2_end):
		o1 = self.orientation(p_1_begin,p_1_end,p_2_begin)
		o2 = self.orientation(p_1_begin,p_1_end,p_2_end)
		o3 = self.orientation(p_2_begin,p_2_end,p_1_begin)
		o4 = self.orientation(p_2_begin,p_2_end,p_1_end)

		if(o1!=o2 and o3!=o4):
			return True

		if(o1==0 and self.onSegment(p_1_begin,p_2_begin,p_1_end)):
			return True

		if(o2==0 and self.onSegment(p_1_begin,p_2_begin,p_1_end)):
			return True

		if(o3==0 and self.onSegment(p_2_begin,p_1_begin,p_2_end)):
			return True

		if(o4==0 and self.onSegment(p_2_begin,p_1_end,p_2_end)):
			return True

		return False

	def CrossesObstacles(self,p_1,p_2,obstacles):
		CrossesObstacle = False
		for obstacle in obstacles:
			if(self.doIntersect(p_1,p_2,obstacle[0],obstacle[-1])):
				return True

	def LiesBeneathObstacles(self,p,obstacles):

		for obstacle in obstacles:
			for point in obstacle:
				if(self.isInObstCircle(p,point)):
					return True

	def connect(self,vertex_1,vertex_2,obstacles,max_edge_px):
		candidate_dist = math.hypot(vertex_1.position[0]-vertex_2.position[0],vertex_1.position[1]-vertex_2.position[1])
		if(candidate_dist <= max_edge_px and not self.CrossesObstacles(vertex_1.position,vertex_2.position,obstacles) and not self.LiesBeneathObstacles(vertex_2.position,obstacles)):
			new_vertex = vertex_2.position
		else:
			
			new_vertex = [vertex_1.position[0]
									-max_edge_px*math.cos(math.atan2(vertex_1.position[1]-vertex_2.position[1],vertex_1.position[0]-vertex_2.position[0]))
										,vertex_1.position[1]-
											max_edge_px*math.sin(math.atan2(vertex_1.position[1]-vertex_2.position[1],vertex_1.position[0]-vertex_2.position[0]))]
		
			new_vertex = [int(new_vertex[0]),int(new_vertex[1])]
			for obstacle in obstacles:
				InObstacle=True
				CrossesObstacle =True
				while InObstacle or CrossesObstacle:
					new_vertex = [vertex_1.position[0]
									-max_edge_px*math.cos(math.atan2(vertex_1.position[1]-vertex_2.position[1],vertex_1.position[0]-vertex_2.position[0]))
										,vertex_1.position[1]-
											max_edge_px*math.sin(math.atan2(vertex_1.position[1]-vertex_2.position[1],vertex_1.position[0]-vertex_2.position[0]))]
					CrossesObstacle=self.doIntersect(new_vertex,vertex_1.position,obstacle[0],obstacle[-1])
					
					InObstacle=False
					for point in obstacle:
						if(self.isInObstCircle(new_vertex,point)):
							InObstacle=True

					max_edge_px-=1
		return new_vertex

	def onMouse(self,event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONUP:
			self.clicked = True	
		cv2.destroyWindow(self.win_name)

def build_wall(begin_coord,end_coord,n_of_points):
	x_spacing = (end_coord[0]-begin_coord[0])/(n_of_points+1)
	y_spacing = (end_coord[1]-begin_coord[1])/(n_of_points+1)

	return [[int(begin_coord[0]+i*x_spacing),int(begin_coord[1]+i*y_spacing)]
				for i in range(1,n_of_points+1)]

def main():
	planner = RRT_PLANNER()
	img = cv2.imread("floor_without_p.png",1)
	height, width = img.shape[:2]
	wall1 = build_wall([0,100],[width-100,100],200)
	wall2 = build_wall([width,200],[100,200],200)
	wall3 = build_wall([0,300],[width-100,300],200)
	#wall4 = build_wall([300,350],[200,250],200)
	planner.get_path(img,[25,10],[wall1,wall2,wall3],[400,400],5000,50)

if __name__ == '__main__':
	main()