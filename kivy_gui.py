# coding:utf-8
from kivy.app import App
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from kivy.uix.scatter import Scatter
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.scatterlayout import ScatterLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.graphics import Color, Ellipse, Line
from kivy.graphics.instructions import InstructionGroup
from kivy.config import Config
import numpy as np
import math
import cv2 as cv
import time
import rtr_planner as planner
import Mapping as mapping
import Socket.Client as client
import Socket.Server as server

#cam res 640 480
WINH = 600
WINW = 1400
global cam_res
global cam_pose
cam_res = (640,480)
cam_pose = (30,60)
Config.set('graphics', 'width', str(WINW))
Config.set('graphics', 'height', str(WINH))
global is_drawing
path_done = False
is_drawing = False
dest_exsist = False
is_drawing_obst = False
dest = "Not yet entered"
px_to_m = 3.60 / math.hypot(cam_res[0],cam_res[1])
obst_lines = []

class KivyCamera(Image):
	def __init__(self, capture, fps, **kwargs):
		super(KivyCamera, self).__init__(**kwargs)
		self.capture = capture

		Clock.schedule_interval(self.update, 1.0 / fps)
		self.cam_mtx = np.load("cam_mtx.npy")
		self.dist_coefs = np.load("dist_coefs.npy")

	def update(self, dt):
		global path_done
		ret, self.frame = self.capture.read()

		#print(frame.shape[1],frame.shape[0])
		if ret:
			# convert it to texture
			if(path_done):
				buf1 = cv.flip(self.draw_path(self.clear_distort(self.frame)), 0)
			else:
				buf1 = cv.flip(self.clear_distort(self.frame), 0)
			buf = buf1.tostring()
			image_texture = Texture.create(
				size=(self.frame.shape[1],self.frame.shape[0] ), colorfmt='bgr')
			image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
			# display image from the texture

			self.texture = image_texture



	def draw_path(self,img):
		#print("im here")
		global path_done
		global path
		global rt_tree

		if(path is None):
			font = cv.FONT_HERSHEY_SIMPLEX
			cv.putText(img,"Path not found",(30,30), font, 1,(0,0,255),2,cv.LINE_AA)
			return img

		if(path_done):
			return rt_tree.Tree_root.draw_path([],path,img)
		else:
			return img

	def clear_distort(self,img):
		h,  w = img.shape[:2]
		newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.cam_mtx, self.dist_coefs, (w,h), 1, (w,h))
		#undistort
		mapx, mapy = cv.initUndistortRectifyMap(self.cam_mtx, self.dist_coefs, None, newcameramtx, (w,h), 5)
		dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
		#x, y, w, h = roi
		#dst = dst[y:y+h, x:x+w]
		return dst


class CamApp(App):


	def build(self):
		global dest
		global dest_label
		global button_layout

		self.capture = cv.VideoCapture(1)
		self.capture.set(cv.CAP_PROP_AUTOFOCUS, 0)
		self.capture.set(cv.CAP_PROP_FOCUS, 4.5)
		self.my_camera = KivyCamera(capture=self.capture, fps=30)

		button_layout = BoxLayout(orientation = 'vertical')
		general_layout = BoxLayout(orientation='horizontal')
		teleop_layout = BoxLayout(orientation = 'vertical')
		self.vel_label = Label(text = "Velocity: ")
		self.angle_vel_label = Label(text = "Angular Velocity: ")
		self.dist_left_label = Label(text = "Distance left: ")
		teleop_layout.add_widget(self.vel_label)
		teleop_layout.add_widget(self.angle_vel_label)
		teleop_layout.add_widget(self.dist_left_label)
		self.cam = FloatLayout()
		clear_dest_btn = Button(text = 'Clear destination',size_hint=(1, 0.1),pos_hint = {'x':.0,'top': 1})
		clear_obst_btn = Button(text = 'Clear obstacles',size_hint=(1, 0.1),pos_hint = {'x':.0,'top': 1})
		clear_obst_btn.bind(on_release=self.clear_obstacles)
		self.drawbtn = Button(text = 'Construct path',size_hint=(1, 0.1),pos_hint = {'x':.0,'top':.5})
		self.start_obst_drawing_btn = Button(text = 'Start Obstacles drawing',size_hint=(1, 0.1))
		self.start_obst_drawing_btn.bind(on_release=self.obstacles_drawing_mode)
		self.cam.add_widget(self.my_camera)
		self.painter = self.MyPaintWidget()
		dest_label = Label(text ="Destination point: " + str(dest))
		clear_dest_btn.bind(on_release = self.clear_dest_point)
		self.drawbtn.bind(on_release = self.draw_path_btn)
		self.cam.add_widget(self.painter)
		button_layout.add_widget(clear_dest_btn)
		button_layout.add_widget(clear_obst_btn)
		button_layout.add_widget(dest_label)
		button_layout.add_widget(self.start_obst_drawing_btn)
		button_layout.add_widget(self.drawbtn)
		button_layout.add_widget(teleop_layout)
		general_layout.add_widget(self.cam)
		general_layout.add_widget(button_layout)

		#port = 3000
		#self.serv = server.Server(port)
		#print(self.serv.ready())

		event = Clock.schedule_interval(self.update, 1.)
		self.robot_pose_old = [[0,0],0]
		return general_layout

	def clear_dest_point(self, obj):
		global dest_exsist
		global dest_label
		dest_label.text = "Not yet entered"
		dest_exsist = False
		print(self.my_camera.pos)
		print(self.my_camera.size)
		self.painter.canvas.clear()
		dest = "Not yet entered"
		# with self.painter.canvas:
		# 	Color(1,0,0)
		# 	for n in range(0,len(all_obst_points)-1):
		# 		if(all_obst_points[n]==[0,0] or all_obst_points[n+1]==[0,0]):
		# 			continue
		# 		Line(points = [all_obst_points[n][0],all_obst_points[n][1],all_obst_points[n+1][0],all_obst_points[n+1][1]],width = 3)


	def clear_obstacles(self,obj):
		global dest


		self.painter.canvas.clear()
		with self.painter.canvas:
			Color(1,0,0)
			d=10.
			#print(dest)
			if(not type(dest) is str):
				Ellipse(pos=((dest[0]/px_to_m-d/2),cam_pose[1]+dest[1]/px_to_m-d/2),size=(d,d))

	def update(self,dt):

		robot_pose_new = mapping.Mapping().find_car(self.my_camera.frame)
		print(robot_pose_new)
		print(self.robot_pose_old)
		if(robot_pose_new[0] is not False and robot_pose_new[1] is not False):
			velocity = (robot_pose_new[0][0]-self.robot_pose_old[0][0])/dt
			angular_velocity = (robot_pose_new[0][1] - self.robot_pose_old[0][1])/dt
			print("Vel: " + str(velocity))
			print("Ang: " + str(angular_velocity))
			self.vel_label.text =  "Velocity: " + str(velocity)
			self.angle_vel_label.text =  "Angular Velocity: " +str(angular_velocity)
			self.robot_pose_old =  robot_pose_new
			#self.serv.send([robot_pose_new[0],robot_pose_new[1]])

			#self.serv.send([robot_pose_new[0],robot_pose_new[1]])


	def draw_path_btn(self,obj):
		global is_drawing
		global dest
		global path_done
		global path
		global rt_tree
		obst = []
		path_done =False


		robot = planner.Robot([50,50],75,40)
		rt_tree = planner.RTR_PLANNER(robot,np.zeros(shape=(480,640)))
		robot_pose=  mapping.Mapping().find_car(self.my_camera.frame)
		obst = mapping.Mapping().get_map(self.my_camera.frame)
		print("Robot pose: " + str(robot_pose))
		print("Obstacles: " + str(obst))
		if(robot_pose[0]==False or robot_pose[1]==False):
			print("car is not detected")
		elif(type(dest) is str):
			print("enter dest point first!")
		else:
			is_drawing = not is_drawing
			q_root = planner.Tree.q(robot_pose[0],robot_pose[1])
			q_root.parent = None
			q_end = planner.Tree.q([int(dest[0]/px_to_m),cam_res[1]-int(dest[1]/px_to_m)],-self.painter.angle*math.pi/180)
			print("POSE: ")
			print(q_end.x,q_end.y,-self.painter.angle*math.pi/180)
			q_end.parent = None
			path = rt_tree.construct(q_root,q_end,obst,50,150)

			#FIXME
			transform_path = path
			#self.serv.send(transform_path)
			print(path)
			path_done = True


	def obstacles_drawing_mode(self,obj):
		global is_drawing_obst
		is_drawing_obst = not is_drawing_obst
		if(is_drawing_obst):
			self.start_obst_drawing_btn.text = "Stop Obstacles drawing"
		else:
			#obst = all_obst_points
			##all_obst_points = []
			#all_obst_points.append(obst)
			#print(all_obst_points)
			self.start_obst_drawing_btn.text = "Start Obstacles drawing"

	class MyPaintWidget(Widget):


		def on_touch_down(self, touch):
			global dest_exsist
			global dest
			global dest_label
			global is_drawing_obst
			self.obstacles = []
			#global self.dest_px
			#global angle

			self.second_points = []
			#print(touch.x,touch.y)
			if(not is_drawing_obst and touch.x > cam_pose[0] and touch.y > cam_pose[1] and touch.x < cam_pose[0]+cam_res[0] and touch.y < cam_pose[1]+cam_res[1] and not dest_exsist):
				with self.canvas:
					self.lines = InstructionGroup()
					dest_exsist = True
					self.dest_px = [touch.x,touch.y]
					dest = [math.ceil(px_to_m*(touch.x-cam_pose[0])*100)/100,math.ceil(px_to_m*(touch.y-cam_pose[1])*100)/100]
					Color(1,0,0)
					self.angle = 0
					d=10.
					Ellipse(pos=(touch.x - d/2,touch.y-d/2),size=(d,d))
					dest_label.text ="Destination point: " + str(dest) +"\n" + "Angle: " + str(self.angle)
					#self.touch.ud['line'] = Line(points=(touch.x, touch.y))
			elif(is_drawing_obst):
				self.obstacles.append([touch.x,touch.y])

		# def get_dest_xy(self):
		# 	return self.dest
        #
		def on_touch_up(self,touch):
			global obstacles
			global all_obst_points
			global obst_lines
			if(is_drawing_obst):
				obst_lines.append([self.obstacles[0],self.obstacles[-1]])
				print("Obst_lines :{ " + str(obst_lines) + " }")

		def on_touch_move(self,touch):
			global dest_label
			global is_drawing_obst


			def areOn1Line(x1,y1,x2,y2,x3,y3):
				if(x2-x1==0 or y2-y1==0):
					return True
				return ((x3-x1)/(x2-x1)==(y3-y1)/(y2-y1))

			def coord_to_angle(x1,y1,x2,y2):
				return	math.atan2(y1 - y2, x1 - x2) / math.pi * 180

			def rasst(x1,y1,x2,y2):
				return math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2))

			if(is_drawing_obst):
				if(touch.x > cam_pose[0] and touch.y > cam_pose[1] and touch.x < cam_pose[0]+cam_res[0] and touch.y < cam_pose[1]+cam_res[1]):
					self.obstacles.append([touch.x,touch.y])
					with self.canvas:
						Color(1,0,0)
						Line(points = [self.obstacles[-1][0],self.obstacles[-1][1],self.obstacles[-2][0],self.obstacles[-2][1]],size = (10.,10.))


			else:
				self.second_points.append([touch.x,touch.y])
				with self.canvas:
					if(touch.x > cam_pose[0] and touch.y > cam_pose[1] and touch.x < cam_pose[0]+cam_res[0] and touch.y < cam_pose[1]+cam_res[1]):
						r=50
						circle_x = self.dest_px[0]-r*math.cos(coord_to_angle(self.dest_px[0],self.dest_px[1],touch.x,touch.y)*math.pi/180)
						touch_cos = math.cos(coord_to_angle(self.dest_px[0],self.dest_px[1],touch.x,touch.y)*math.pi/180)
						circle_y = self.dest_px[1]-r*math.sin(coord_to_angle(self.dest_px[0],self.dest_px[1],touch.x,touch.y)*math.pi/180)
						touch_sin = math.sin(coord_to_angle(self.dest_px[0],self.dest_px[1],touch.x,touch.y)*math.pi/180)
						circle_x_arrow = circle_x+20*(math.cos(math.pi/6+math.atan2(touch_sin,touch_cos)))
						circle_y_arrow = circle_y+20*(math.sin(math.pi/6+math.atan2(touch_sin,touch_cos)))
						circle_x_arrow_2 = circle_x+20*(math.cos(math.pi/6-math.atan2(touch_sin,touch_cos)))
						circle_y_arrow_2 = circle_y-20*(math.sin(math.pi/6-math.atan2(touch_sin,touch_cos)))

						if(len(self.second_points)>1):
							if(areOn1Line(self.dest_px[0],self.dest_px[1],
											self.second_points[-1][0],self.second_points[-1][1],
												self.second_points[-2][0],self.second_points[-2][1])):
								#Line(points = [self.dest_px[0],self.dest_px[1],touch.x,touch.y])
								Line(points = [self.dest_px[0],self.dest_px[1],circle_x,circle_y])
								Line(points = [circle_x,circle_y,circle_x_arrow,circle_y_arrow])
								Line(points = [circle_x,circle_y,circle_x_arrow_2,circle_y_arrow_2])
							else:
								self.canvas.clear()
								Color(1,0,0)
								Color(1,0,0)
								Line(points = [self.dest_px[0],self.dest_px[1],circle_x,circle_y])
								#Line(points = [circle_x,circle_y,circle_x+(-10*touch_cos+(10)*touch_sin),circle_y+((10)*touch_cos+10*touch_sin)])
								Line(points = [circle_x,circle_y,circle_x_arrow,circle_y_arrow])
								Line(points = [circle_x,circle_y,circle_x_arrow_2,circle_y_arrow_2])
								# Line(points = [self.dest_px[0],self.dest_px[1],
								# 	self.dest_px[0]+20*math.copysign(1,math.atan2(self.dest_px[0]-touch.x,self.dest_px[1]-touch.y)),
								# 	self.dest_px[1]+20*math.copysign(1,math.atan2(self.dest_px[0]-touch.x,self.dest_px[1]-touch.y))])
								d=10.
								Ellipse(pos=(self.dest_px[0]- d/2,self.dest_px[1]-d/2),size=(d,d))

						elif(len(self.second_points)==1):
							Line(points = [self.dest_px[0],self.dest_px[1],touch.x,touch.y])
						self.angle = coord_to_angle(touch.x,touch.y,self.dest_px[0],self.dest_px[1])
						dest_label.text ="Destination point: " + str(dest) +"\n" + "Angle: " + str(math.ceil(self.angle*100)/100)

	def on_stop(self):
		#without this, app will not exit even if the window is closed
		self.capture.release()


if __name__ == '__main__':
	CamApp().run()
