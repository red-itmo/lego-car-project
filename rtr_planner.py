import random
import math
import numpy as np
import cv2
import utility as util
import copy
import Car_detection as detect 
random.seed()

class Robot:

	def __init__(self,p1,width,height):
		self.pos = [0,0]
		self.width = width
		self.height = height
		self.centr_p1 = p1
		

	def get4pointsofrobot(self,q):
		
		r1 = math.sqrt(math.pow(self.width/4,2)+math.pow(self.height/2,2))
		r2 = math.sqrt(math.pow(self.width/4,2)+math.pow(self.height/2,2))
		r3 = math.sqrt(math.pow(self.width/4*3,2)+math.pow(self.height/2,2))
		r4 = math.sqrt(math.pow(self.width/4*3,2)+math.pow(self.height/2,2))


		angle1 = math.pi-math.acos(self.width/4/r1)
		angle2 = -angle1
		angle3 = math.acos(self.width*3/4/r3)
		angle4 = -angle3

		point1 = [q.x + r1*math.cos(angle1+q.theta),
					q.y + r1*math.sin(angle1+q.theta)]
		point2 = [q.x + r2*math.cos(angle2+q.theta),
					q.y + r2*math.sin(angle2+q.theta)]
		point4 = [q.x + r3*math.cos(angle3+q.theta),
					q.y + r3*math.sin(angle3+q.theta)]
		point3 = [q.x + r4*math.cos(angle4+q.theta),
					q.y + r4*math.sin(angle4+q.theta)]

		return [point1,point2,point3,point4]

	def ifCrash(self,q,obstacles,img,primitive,direction):
		points_of_robot = self.get4pointsofrobot(q)
		Crash = False
		turn_radius = math.sqrt(2)*self.height/2
		#turn_radius = 5
		if(primitive == "translate" and direction=="forward"):
			#print(points_of_robot[3])
			points_of_robot[3] =[points_of_robot[3][0]+turn_radius*math.cos(q.theta),points_of_robot[3][1]+turn_radius*math.sin(q.theta)] 
			points_of_robot[2] =[points_of_robot[2][0]+turn_radius*math.cos(q.theta),points_of_robot[2][1]+turn_radius*math.sin(q.theta)]  
		if(primitive == "translate" and direction=="backward"):
			points_of_robot[0] =[points_of_robot[0][0]-turn_radius*math.cos(q.theta),points_of_robot[0][1]-turn_radius*math.sin(q.theta)] 
			points_of_robot[1] =[points_of_robot[1][0]-turn_radius*math.cos(q.theta),points_of_robot[1][1]-turn_radius*math.sin(q.theta)] 


		Crash = util.CrossesObstacles(points_of_robot[0],points_of_robot[-1],obstacles)
		if(Crash):
			return True
		for i in range(len(points_of_robot)-1):
			Crash = util.CrossesObstacles(points_of_robot[i],points_of_robot[i+1],obstacles)
			if(Crash):
				return True
		
		
		if(not util.inRangeOfImg(points_of_robot,img)):
			return True
		return Crash

class Tree:

	class q:

		def __init__(self,position,theta):
			self.x = position[0]
			self.y = position[1]
			self.theta = theta
			self.parent = 1
			self.child = []

		def Translate(self,direction,cost):

			if(direction == "forward"):
				self.x += cost*math.cos(self.theta) 
				self.y += cost*math.sin(self.theta)
			elif(direction == "backward"):
				self.x -= cost*math.cos(self.theta) 
				self.y -= cost*math.sin(self.theta)
			else:
				print("Specify directionection by 'forward' or 'backward'")

		def Rotate(self,angle):
			self.theta += angle

		def __str__(self):
			return ("|" +str(self.x) + " " + str(self.y) + " " + str(self.theta) + "|" ) 



	class TCI:
		
		def __init__(self,q_begin,q_end):
			self.q_begin = q_begin
			self.q_end = q_end

	class RCI:

		def __init__(self,q_begin,q_end):
			self.q_begin = q_begin
			self.q_end = q_end

	def __init__(self,Robot,img):
		self.edges = []
		self.vertexes = []
		self.robot = Robot
		self.img = img

	def AddVertex(self,q_new):
		self.vertexes.append(copy.deepcopy(q_new))

	def AddEdge(self,qnear,qnew):
		self.edges.append([copy.deepcopy(qnear),copy.deepcopy(qnew)])

	def Init(self,q,obstacles):
		self.AddVertex(q)

		print(q)
		TCI = self.TCIExtend(q,"forward",obstacles)
		q.child.append(TCI.q_end)
		TCI.q_end.parent = q
		self.AddVertex(TCI.q_end)
		self.AddEdge(q,TCI)
		TCI = self.TCIExtend(q,"backward",obstacles)
		q.child.append(TCI.q_end)
		TCI.q_end.parent = q
		self.AddVertex(TCI.q_end)
		self.AddEdge(q,TCI)

	def Extend(self,q,turndirection,obstacles):
		RCI,collision = self.RCIExtend(q,turndirection,obstacles)
		q.child.append(RCI.q_end)
		RCI.q_end.parent = q
		self.AddVertex(RCI.q_end)
		self.AddEdge(q,RCI)
		TCI = self.TCIExtend(RCI.q_end,"forward",obstacles)
		q.child.append(TCI.q_end)
		TCI.q_end.parent = q
		self.AddVertex(TCI.q_end)
		self.AddEdge(RCI.q_end,TCI)
		TCI = self.TCIExtend(RCI.q_end,"backward",obstacles)
		q.child.append(TCI.q_end)
		TCI.q_end.parent = q
		self.AddVertex(TCI.q_end)
		self.AddEdge(RCI.q_end,TCI)

		return collision

	def TCIExtend(self,q,direction,obstacles):
		q_begin = copy.deepcopy(q)
		q_end = copy.deepcopy(q)
		while not self.robot.ifCrash(q_end,obstacles,self.img,"translate",direction):
			q_end.Translate(direction,1)
		return self.TCI(q_begin,q_end)

	def RCIExtend(self,q,angle,obstacles):
		collision = False
		q_end = copy.deepcopy(q)
		q_begin = copy.deepcopy(q)
		rez_angle = 0
		while abs(rez_angle) < abs(angle) and not collision:
			rez_angle += math.copysign(1,angle)*0.1
			q_end.Rotate(math.copysign(1,angle)*0.1)
			collision = self.robot.ifCrash(q_end,obstacles,self.img,"rotate","backward")


		if(collision):
			q_end.theta -= math.copysign(1,angle)*0.1

		return (self.RCI(q_begin,q_end),collision)

	def NearestNeighbor(self,new_vertex):

		min_dist = None
		min_q = None
		orient = None
		true_edge = None

		for edge in self.edges:
			if(edge[0].x == edge[1].q_end.x and edge[0].y == edge[1].q_end.y):
				candidate_q = [edge[1].q_end.x,edge[1].q_end.y]

			else:
				candidate_q = util.ClosestPoint(edge[0],edge[1].q_end,self.q(new_vertex,0))
			candidate_dist = math.hypot(new_vertex[0] - candidate_q[0],new_vertex[1]-candidate_q[1])
			if min_dist == None or candidate_dist < min_dist:
				true_edge = edge
				min_dist = candidate_dist
				min_q = candidate_q 
				orient = edge[0].theta 
		Q = self.q(min_q,orient)
		Q.parent = true_edge[0]
		return Q

	def draw_all(self,obstacles,color):
		img = self.img
		n = 0
		for vertex in self.vertexes:

			cv2.circle(img,(math.ceil(vertex.x),math.ceil(vertex.y)),2,(0,0,255))
			if(n%50==0 or True):
				self.draw_robot(img,vertex,color)
		for edge in self.edges:
			cv2.line(img,(math.ceil(edge[0].x),math.ceil(edge[0].y)),(math.ceil(edge[1].q_end.x),math.ceil(edge[1].q_end.y)),(255,0,0))
		for obstacle in obstacles:
			for i in range(len(obstacle)-1):
				cv2.line(img,(obstacle[i][0],obstacle[i][1]),(obstacle[i+1][0],obstacle[i+1][1]),(0,255,0))
		
		win_name = "Win1"
		cv2.namedWindow(win_name)
		cv2.imshow(win_name,img)
		cv2.waitKey(0)

	def draw_robot(self,img,q,color):
		p_for_line = self.robot.get4pointsofrobot(q)
		cv2.circle(img,(int(q.x),int(q.y)),5,(255,0,0),-1)	
		for i in range(len(p_for_line)-1):
			cv2.line(img,(int(p_for_line[i][0]),int(p_for_line[i][1])),(int(p_for_line[i+1][0]),int(p_for_line[i+1][1])),color)				
		cv2.line(img,(int(p_for_line[0][0]),int(p_for_line[0][1])),(int(p_for_line[-1][0]),int(p_for_line[-1][1])),color)
		return img

	def draw_path(self,obstacles,path,img):
		#img = self.img
		for obstacle in obstacles:
			for i in range(len(obstacle)-1):
				cv2.line(img,(obstacle[i][0],obstacle[i][1]),(obstacle[i+1][0],obstacle[i+1][1]),(0,255,0))
		for n in range(len(path)-1):
			cv2.line(img,(math.ceil(path[n].x),math.ceil(path[n].y)),(math.ceil(path[n+1].x),math.ceil(path[n+1].y)),(255,0,0))
			font = cv2.FONT_HERSHEY_SIMPLEX
			#cv2.putText(img,str(n),(int(path[n].x),int(path[n].y)), font, 1,(255,255,255),2,cv2.LINE_AA)
			#if(path[n+1].theta == path[n].theta):
			self.draw_robot(img,path[n],(0,0,255))
		self.draw_robot(img,path[-1],(0,0,255))
		return img

class RTR_PLANNER:

	def __init__(self,Robot,img):
		self.win_name = "Win1"
		self.Tree_end = Tree(Robot,img)
		self.Tree_root = Tree(Robot,img)
		self.dims = [img.shape[:2][1],img.shape[:2][0]]

	def construct(self,q_init,q_end,obstacles,max_expand_dist = 50,n_of_iterations = 50):
		self.Tree_root.Init(q_init,obstacles)
		self.Tree_end.Init(q_end,obstacles)		
		for k in range(n_of_iterations):
			P_g_init = self.RandomPos(self.dims)
			P_g_end = self.RandomPos(self.dims)
			q_near_init = self.Tree_root.NearestNeighbor(P_g_init)
			q_near_end = self.Tree_end.NearestNeighbor(P_g_end)
			turndirection_init = self.MinTurndirection(q_near_init,P_g_init)
			turndirection_end = self.MinTurndirection(q_near_end,P_g_end)
			collision_init = self.Tree_root.Extend(q_near_init,turndirection_init,obstacles)
			collision_end = self.Tree_end.Extend(q_near_end,turndirection_end,obstacles)
			if collision_init:
				self.Tree_root.Extend(q_near_init, -math.copysign(1,turndirection_init)* (2*math.pi - abs(turndirection_init)),obstacles)
			if collision_end:
				self.Tree_end.Extend(q_near_end, -math.copysign(1,turndirection_end)* (2*math.pi - abs(turndirection_end)),obstacles)
			n,goal_achieved = self.checkGoal(self.Tree_root.edges[-1],self.Tree_end.edges,obstacles)
		
			if(goal_achieved):

				path = self.getpath(q_init,self.Tree_root.edges[-1],q_end,self.Tree_end.edges[n])
				#self.Tree_root.draw_path(obstacles,path)
				path = self.correct_angles_in_path(path)
				# img = self.Tree_root.draw_path(obstacles,path,self.Tree_root.img)
				# win_name = "Win1"
				# cv2.namedWindow(win_name)
				# cv2.imshow(win_name,img)
				# cv2.waitKey(0)
				print("Number of iterations: " + str(k))
				return path
		#self.Tree_root.draw_all(obstacles,(255,0,0))
		#self.Tree_end.draw_all(obstacles,(255,255,255))

		return None

	def checkGoal(self,edge_init,edges_end,obstacles):
		for n in range(len(edges_end)):
			if(type(edges_end[n][1]) is Tree.RCI or type(edge_init[1]) is Tree.RCI):
				continue
			if(util.doIntersect([edge_init[1].q_begin.x,edge_init[1].q_begin.y],[edge_init[1].q_end.x,edge_init[1].q_end.y],
									[edges_end[n][1].q_begin.x,edges_end[n][1].q_begin.y],[edges_end[n][1].q_end.x,edges_end[n][1].q_end.y])):
				intersection = util.line_intersection([[edge_init[0].x,edge_init[0].y],[edge_init[1].q_end.x,edge_init[1].q_end.y]],
													[[edges_end[n][0].x,edges_end[n][0].y],[edges_end[n][1].q_end.x,edges_end[n][1].q_end.y]])
				q_inter = Tree.q(intersection,edge_init[1].q_end.theta)
				angle = self.MinTurndirection(q_inter,[edges_end[n][1].q_end.x,edges_end[n][1].q_end.y])
				_,collision = self.Tree_root.RCIExtend(q_inter,angle,obstacles)
				if(not collision):
					return n,True
		return None,False

	def getpath(self,q_init,edge_init,q_end,edge_end):
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
		
		intersection = util.line_intersection([[edge_init[0].x,edge_init[0].y],[edge_init[1].q_end.x,edge_init[1].q_end.y]],
												[[edge_end[0].x,edge_end[0].y],[edge_end[1].q_end.x,edge_end[1].q_end.y]])
		path1.append(Tree.q(intersection,path1[-1].theta))
		#path2 = path2[::-1]
		path1.extend(path2)

		return path1

	def find(self,q_begin,tree):
		for edge in tree.edges:
			if(type(edge[1]) is Tree.RCI):
				continue
			if(edge[1].q_end == q_begin and edge[1].q_begin!=q_begin):

				return edge[1]

	def correct_angles_in_path(self,path):
		correct_path = []
		for n in range(len(path)-1):
			if(n==0):
				correct_path.append(path[0])
				continue
			#if(path[n+1].theta == path[n].theta):
			correct_path.append(Tree.q([path[n].x,path[n].y],path[n].theta + self.MinTurndirection(path[n],[path[n+1].x,path[n+1].y])))
		correct_path.append(Tree.q([path[-1].x,path[-1].y],path[-1].theta))
		#print(correct_path)
		return correct_path

	def MinTurndirection(self,q_nearest,random_point):
		x = random_point[0] - q_nearest.x
		y = random_point[1] - q_nearest.y

		dx = math.cos(q_nearest.theta)
		dy = math.sin(q_nearest.theta)
		angle = math.atan2(x*dy - y*dx,x*dx+y*dy)

		return -angle

	def RandomPos(self,dims):
		return [random.randint(0,dims[0]),random.randint(0,dims[1])]




# def main():
# 	img = cv2.imread("35.png",1)
# 	robot_pose=  detect.Car_detection().find_car(img)
# 	print(robot_pose)
# 	q_root = Tree.q(robot_pose[0],robot_pose[1])
# 	q_root.parent = None
# 	q_end = Tree.q([500,400],0)
# 	q_end.parent = None
# 	robot = Robot([q_root.x,q_root.y],80,60)
# 	rt_tree = RTR_PLANNER(robot,img)

# 	# wall6 = util.build_wall([0,100],[img.shape[:2][1]-300,100],200)
# 	# wall5 = util.build_wall([img.shape[:2][1],200],[300,200],200)
# 	# wall7 = util.build_wall([0,300],[img.shape[:2][1]-300,300],200)
# 	#wall8 = util.build_wall([img.shape[:2][1]-100,350],[img.shape[:2][1]-200,450],200)

# 	#obstacles = [wall6,wall5,wall7]
# 	obstacles = []
# 	path1 = rt_tree.construct(q_root,q_end,obstacles,50,200)
# 	print(path1)


# if __name__ == '__main__':
# 	main()

