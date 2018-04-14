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
import src.rtr_planner as planner
import src.Mapping as mapping
import src.Socket.Client as client
import src.Socket.Server as server
import src.Auxilary as aux

global cam_res
global cam_pose
global is_drawing


# --------------------------------------------
# WARNING
# CHANGING BELOW HEIGHT AND WIDTH OF THE SCREEN
# CAN LEAD TO STRANGE BEHAVIOUR
# --------------------------------------------
WINH = 600
WINW = 1400

cam_res = (640, 480)
# pose of the camera widget in the window
cam_pose = (30, 60)
Config.set('graphics', 'width', str(WINW))
Config.set('graphics', 'height', str(WINH))

path_done = False
is_drawing = False
dest_exsist = False
is_drawing_obst = False
dest = "Not yet entered"
# 3.60 is diagonal of the floor under the camera
px_to_m = 3.60 / math.hypot(cam_res[0], cam_res[1])
obst_lines = []
fh = open("logs_gui.txt", "w")

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
		if ret:
			# convert it to texture
			if (path_done):
				buf1 = cv.flip(self.draw_path(self.clear_distort(self.frame), True), 0)
			else:
				buf1 = cv.flip(self.clear_distort(self.frame), 0)
			buf = buf1.tostring()
			image_texture = Texture.create(size=(self.frame.shape[1],
											self.frame.shape[0]), colorfmt='bgr')
			image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
			# display image from the texture
			self.texture = image_texture


	def draw_path(self, img, mode):
		global path_done
		global path
		global rt_tree
		if (path is None or path is False):
			font = cv.FONT_HERSHEY_SIMPLEX
			cv.putText(img, "Path not found", (30, 30), font, 1, (0, 0, 255), 2, cv.LINE_AA)
			return img

		if (path_done):
			return rt_tree.Tree_root.draw_path([], path, img, mode)
		else:
			return img


	def clear_distort(self, img):
		h, w = img.shape[:2]
		newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.cam_mtx, self.dist_coefs, (w, h), 1, (w, h))
		# undistort
		mapx, mapy = cv.initUndistortRectifyMap(self.cam_mtx, self.dist_coefs, None, newcameramtx, (w, h), 5)
		dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
		return dst


class CamApp(App):


	def build(self):
		global dest
		global dest_label
		global button_layout
		self.capture = cv.VideoCapture(0)
		self.my_camera = KivyCamera(capture=self.capture, fps=30)
		button_layout = BoxLayout(orientation='vertical')
		general_layout = BoxLayout(orientation='horizontal')
		teleop_layout = BoxLayout(orientation='vertical')
		self.vel_label = Label(text="Velocity: ")
		self.angle_vel_label = Label(text="Angular Velocity: ")
		self.dist_left_label = Label(text="Distance left: ")
		teleop_layout.add_widget(self.vel_label)
		teleop_layout.add_widget(self.angle_vel_label)
		teleop_layout.add_widget(self.dist_left_label)
		self.cam = FloatLayout()
		clear_dest_btn = Button(text='Clear destination', size_hint=(1, 0.1), pos_hint={'x': .0, 'top': 1})
		clear_obst_btn = Button(text='Clear obstacles', size_hint=(1, 0.1), pos_hint={'x': .0, 'top': 1})
		clear_obst_btn.bind(on_release=self.clear_obstacles)
		self.drawbtn = Button(text='Construct path', size_hint=(1, 0.1), pos_hint={'x': .0, 'top': .5})
		self.start_obst_drawing_btn = Button(text='Start Obstacles drawing', size_hint=(1, 0.1))
		self.start_obst_drawing_btn.bind(on_release=self.obstacles_drawing_mode)
		self.cam.add_widget(self.my_camera)
		self.painter = self.MyPaintWidget()
		dest_label = Label(text="Destination point: " + str(dest))
		clear_dest_btn.bind(on_release=self.clear_dest_point)
		self.drawbtn.bind(on_release=self.draw_path_btn)
		self.cam.add_widget(self.painter)
		button_layout.add_widget(clear_dest_btn)
		button_layout.add_widget(clear_obst_btn)
		button_layout.add_widget(dest_label)
		button_layout.add_widget(self.start_obst_drawing_btn)
		button_layout.add_widget(self.drawbtn)
		button_layout.add_widget(teleop_layout)
		general_layout.add_widget(self.cam)
		general_layout.add_widget(button_layout)

		# SRV CONNECTION PART
		self.send = False
		print("Trying to connect to remote server")
		port = 3000
		self.serv = server.Server(port)
		print(self.serv.ready())
		print("Connection esteblished")

		event = Clock.schedule_interval(self.update, 1. / 10)
		self.robot_pose_old = [[0, 0], 0]
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


	def clear_obstacles(self, obj):
		global dest
		self.painter.canvas.clear()
		with self.painter.canvas:
			Color(1, 0, 0)
			d = 10.
			if (not type(dest) is str):
				Ellipse(pos=((dest[0]/px_to_m - d/2), cam_pose[1] + dest[1]/px_to_m - d/2), size=(d, d))


	def update(self, dt):
		global path_done
		robot_pose_new = mapping.Mapping().find_car(self.my_camera.frame)
		if (robot_pose_new[0] is not False and robot_pose_new[1] is not False):
			velocity = (robot_pose_new[0][0] - self.robot_pose_old[0][0]) / dt
			angular_velocity = (robot_pose_new[0][1] - self.robot_pose_old[0][1]) / dt
			self.vel_label.text = "Velocity: " + str(velocity)
			self.angle_vel_label.text = "Angular Velocity: " + str(angular_velocity)
			self.robot_pose_old = robot_pose_new
			# is_sent = self.serv.send([robot_pose_new[0][0] * px_to_m, robot_pose_new[0][1] * px_to_m])
			# print("I AM IN UPDATE")
			# print(is_sent)
			# while not is_sent and self.send:
			# 	is_sent = self.serv.send([robot_pose_new[0][0] * px_to_m, robot_pose_new[0][1] * px_to_m])
			# 	print(is_sent)
			fh.write("%f %f %f\n" % (robot_pose_new[0][0],robot_pose_new[0][1],robot_pose_new[1]))
			if (self.send):
				while True:
						print("update")
						try:
							sent = self.serv.send([robot_pose_new[0][0] * px_to_m, robot_pose_new[0][1] * px_to_m])
						except ConnectionResetError:
							fh.close()
						if sent:
							break

	def draw_path_btn(self, obj):
		global is_drawing
		global dest
		global path_done
		global path
		global rt_tree
		obst = []
		path_done = False
		robot = planner.Robot([50, 50], 75, 40)
		rt_tree = planner.RTR_PLANNER(robot, np.zeros(shape=(480, 640)))
		robot_pose = mapping.Mapping().find_car(self.my_camera.frame)

		# uncomment bellow for mapping from the camera
		# NOT STABLE
		# obst = mapping.Mapping().get_map(self.my_camera.frame)
		obst = []
		img = self.my_camera.frame
		for obstacle in obst:
			for i in range(len(obstacle) - 1):
				cv.line(img, (obstacle[i][0], obstacle[i][1]), (obstacle[i + 1][0], obstacle[i + 1][1]), (0, 255, 0))
		if (robot_pose[0] == False or robot_pose[1] == False):
			print("car is not detected")
		elif (type(dest) is str):
			print("enter dest point first!")
		else:
			is_drawing = not is_drawing
			q_root = planner.Tree.q(robot_pose[0], robot_pose[1])
			q_root.parent = None
			q_end = planner.Tree.q([int(dest[0] / px_to_m), cam_res[1] - int(dest[1] / px_to_m)],
								   	-self.painter.angle * math.pi / 180)
			q_end.parent = None
			path = rt_tree.construct(q_root, q_end, obst, 50, 50)
			path = rt_tree.path_into_m(path)
			transformed_path = rt_tree.transform_path(path, "TTS", obst, img, robot, 5, False)

			if(transformed_path):
				path = rt_tree.path_to_px(transformed_path[2])
				descr = transformed_path[0]
				self.send = False
				# traj_is_sent = self.serv.send([[robot_pose[0][0] * px_to_m, robot_pose[0][1] * px_to_m, -robot_pose[1]],descr])
				# while not traj_is_sent:
				# 	traj_is_sent = self.serv.send([[robot_pose[0][0] * px_to_m, robot_pose[0][1] * px_to_m, -robot_pose[1]],
				# 						descr])

				while not self.serv.send([[robot_pose[0][0] * px_to_m, robot_pose[0][1] * px_to_m, -robot_pose[1]],descr]):
					self.serv.send([[robot_pose[0][0] * px_to_m, robot_pose[0][1] * px_to_m, -robot_pose[1]],descr])
					print("I AM IN A FUCING CYCLE YOU IDIOT")
				self.send = True
			else:
				path = False

			path_done = True


	def obstacles_drawing_mode(self, obj):
		global is_drawing_obst
		is_drawing_obst = not is_drawing_obst
		if (is_drawing_obst):
			self.start_obst_drawing_btn.text = "Stop Obstacles drawing"
		else:
			self.start_obst_drawing_btn.text = "Start Obstacles drawing"


	class MyPaintWidget(Widget):


		def on_touch_down(self, touch):
			global dest_exsist
			global dest
			global dest_label
			global is_drawing_obst
			self.obstacles = []
			self.second_points = []
			if (not is_drawing_obst and touch.x > cam_pose[0] and touch.y > cam_pose[1] and touch.x < cam_pose[0] +
					cam_res[0] and touch.y < cam_pose[1] + cam_res[1] and not dest_exsist):
				with self.canvas:
					self.lines = InstructionGroup()
					dest_exsist = True
					self.dest_px = [touch.x, touch.y]
					dest = [math.ceil(px_to_m * (touch.x - cam_pose[0]) * 100) / 100,
							math.ceil(px_to_m * (touch.y - cam_pose[1]) * 100) / 100]
					Color(1, 0, 0)
					self.angle = 0
					d = 10.
					Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d))
					dest_label.text = "Destination point: " + str(dest) + "\n" + "Angle: " + str(self.angle)
			elif (is_drawing_obst):
				self.obstacles.append([touch.x, touch.y])


		def on_touch_up(self, touch):
			global obstacles
			global all_obst_points
			global obst_lines
			if (is_drawing_obst):
				obst_lines.append([self.obstacles[0], self.obstacles[-1]])


		def on_touch_move(self, touch):
			global dest_label
			global is_drawing_obst
			if (is_drawing_obst):
				if (touch.x > cam_pose[0] and touch.y > cam_pose[1] and touch.x < cam_pose[0] + cam_res[0] and touch.y <
						cam_pose[1] + cam_res[1]):
					self.obstacles.append([touch.x, touch.y])
					with self.canvas:
						Color(1, 0, 0)
						Line(points=[self.obstacles[-1][0], self.obstacles[-1][1], self.obstacles[-2][0],
									 self.obstacles[-2][1]], size=(10., 10.))


			# Below is shit about arrow drawing
			else:
				self.second_points.append([touch.x, touch.y])
				with self.canvas:
					if (touch.x > cam_pose[0] and touch.y > cam_pose[1] and touch.x < cam_pose[0] + cam_res[
						0] and touch.y < cam_pose[1] + cam_res[1]):
						r = 50
						circle_x = self.dest_px[0] - r * math.cos(
							aux.coord_to_angle(self.dest_px[0], self.dest_px[1], touch.x, touch.y) * math.pi / 180)
						touch_cos = math.cos(
							aux.coord_to_angle(self.dest_px[0], self.dest_px[1], touch.x, touch.y) * math.pi / 180)
						circle_y = self.dest_px[1] - r * math.sin(
							aux.coord_to_angle(self.dest_px[0], self.dest_px[1], touch.x, touch.y) * math.pi / 180)
						touch_sin = math.sin(
							aux.coord_to_angle(self.dest_px[0], self.dest_px[1], touch.x, touch.y) * math.pi / 180)
						circle_x_arrow = circle_x + 20 * (math.cos(math.pi / 6 + math.atan2(touch_sin, touch_cos)))
						circle_y_arrow = circle_y + 20 * (math.sin(math.pi / 6 + math.atan2(touch_sin, touch_cos)))
						circle_x_arrow_2 = circle_x + 20 * (math.cos(math.pi / 6 - math.atan2(touch_sin, touch_cos)))
						circle_y_arrow_2 = circle_y - 20 * (math.sin(math.pi / 6 - math.atan2(touch_sin, touch_cos)))

						if (len(self.second_points) > 1):
							if (aux.areOn1Line(self.dest_px[0], self.dest_px[1],
										   self.second_points[-1][0], self.second_points[-1][1],
										   self.second_points[-2][0], self.second_points[-2][1])):
								Line(points=[self.dest_px[0], self.dest_px[1], circle_x, circle_y])
								Line(points=[circle_x, circle_y, circle_x_arrow, circle_y_arrow])
								Line(points=[circle_x, circle_y, circle_x_arrow_2, circle_y_arrow_2])
							else:
								self.canvas.clear()
								Color(1, 0, 0)
								Color(1, 0, 0)
								Line(points=[self.dest_px[0], self.dest_px[1], circle_x, circle_y])
								Line(points=[circle_x, circle_y, circle_x_arrow, circle_y_arrow])
								Line(points=[circle_x, circle_y, circle_x_arrow_2, circle_y_arrow_2])
								d = 10.
								Ellipse(pos=(self.dest_px[0] - d / 2, self.dest_px[1] - d / 2), size=(d, d))

						elif (len(self.second_points) == 1):
							Line(points=[self.dest_px[0], self.dest_px[1], touch.x, touch.y])
						self.angle = aux.coord_to_angle(touch.x, touch.y, self.dest_px[0], self.dest_px[1])
						dest_label.text = "Destination point: " + str(dest) + "\n" + "Angle: " + str(
							math.ceil(self.angle * 100) / 100)


	def on_stop(self):
		# without this, app will not exit even if the window is closed
		self.capture.release()


if __name__ == '__main__':
	CamApp().run()
