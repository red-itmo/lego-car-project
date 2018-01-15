# coding:utf-8
from kivy.app import App
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from kivy.uix.scatter import Scatter
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.graphics import Color, Ellipse, Line
from kivy.config import Config
import numpy as np
import math
import cv2 as cv
import time 

#cam res 640 480
WINH = 720
WINW = 960
global cam_res
global cam_pose
cam_res = (480,360)
cam_pose = (0,180)
Config.set('graphics', 'width', str(WINW))
Config.set('graphics', 'height', str(WINH))
global is_drawing
is_drawing = False
dest_exsist = False
dest = "Not yet entered"
px_to_m = 1

class KivyCamera(Image):
	def __init__(self, capture, fps, **kwargs):
		super(KivyCamera, self).__init__(**kwargs)
		self.capture = capture
		Clock.schedule_interval(self.update, 1.0 / fps)
		self.cam_mtx = np.load("cam_mtx.npy")
		self.dist_coefs = np.load("dist_coefs.npy")

	def update(self, dt):
		ret, frame = self.capture.read()
		
		#print(frame.shape[1],frame.shape[0])
		if ret:
			# convert it to texture
			if(is_drawing):
				buf1 = cv.flip(self.draw_path(self.clear_distort(frame)), 0)
			else:
				buf1 = cv.flip(self.clear_distort(frame), 0)
			buf = buf1.tostring()
			image_texture = Texture.create(
				size=(frame.shape[1],frame.shape[0] ), colorfmt='bgr')
			image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
			# display image from the texture
			self.texture = image_texture


	def draw_path(self,img):
		#print("im here")
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
		self.capture = cv.VideoCapture(0)
		self.my_camera = KivyCamera(capture=self.capture, fps=30)
		button_layout = BoxLayout(orientation = 'vertical')
		general_layout = BoxLayout(orientation = 'horizontal')
		self.cam = FloatLayout()
		clearbtn = Button(text = 'Clear destination',size_hint=(1, 0.1),pos_hint = {'x':.0,'top': 1})
		drawbtn = Button(text = 'Start/stop drawing path',size_hint=(1, 0.1),pos_hint = {'x':.0,'top':.5})
		self.cam.add_widget(self.my_camera)
		self.painter = self.MyPaintWidget()
		dest_label = Label(text ="Destination point: " + str(dest))
		clearbtn.bind(on_release = self.clear_canvas)
		drawbtn.bind(on_release = self.draw_path_btn)
		self.cam.add_widget(self.painter)
		button_layout.add_widget(clearbtn)
		button_layout.add_widget(dest_label)
		button_layout.add_widget(drawbtn)

		general_layout.add_widget(self.cam)
		general_layout.add_widget(button_layout)
		return general_layout

	def clear_canvas(self, obj):
		global dest_exsist
		global dest_label
		dest_label.text = "Not yet entered"
		dest_exsist = False
		self.painter.canvas.clear()

	def draw_path_btn(self,obj):
		global is_drawing
		is_drawing = not is_drawing

	class MyPaintWidget(Widget):

		# def __init__(self):
		# 	self.dest = [0,0]
		# 	self.k = 1
		# 	self.dest_exsist = False

		def on_touch_down(self, touch):
			global dest_exsist
			global dest
			global dest_label
			if(touch.x > cam_pose[0] and touch.y > cam_pose[1] and touch.x < cam_pose[0]+cam_res[0] and touch.y < cam_pose[1]+cam_res[1] and not dest_exsist):
				with self.canvas:
					dest_exsist = True
					dest = [px_to_m*math.ceil(touch.x*100)/100,px_to_m*math.ceil(touch.y*100)/100]
					Color(1,0,0)
					d=10.
					Ellipse(pos=(touch.x - d/2,touch.y-d/2),size=(d,d))
					dest_label.text ="Destination point: " + str(dest)
					#button_layout.add_widget(dest_label)
		# def get_dest_xy(self):
		# 	return self.dest		


	def on_stop(self):
		#without this, app will not exit even if the window is closed
		self.capture.release()


if __name__ == '__main__':
	CamApp().run()