#!/usr/bin/env python3
# coding: utf-8

import cv2
import numpy as np

RESOLUTION = (640, 480)         # Width, Height
FOVS = (43.78, 54.35, 65.47)    # vertical, horizontal, diagonal angles. Logitech 920FullHD (640, 480)
H = 2.85                         # height from floor to camera
D_M = 3.6
D_PX = np.sqrt(RESOLUTION[0] ** 2 + RESOLUTION[1] ** 2)
K = D_M / D_PX
WIDTH_M = K * RESOLUTION[1]
HEIGHT_M = K * RESOLUTION[0]
print("metr in px {}".format(K))


class Marker:

    def __init__(self, cx, cy, cnt, color):
        self.cx = cx
        self.cy = cy
        self.cnt = cnt
        self.color = color


class Camera:

    def __init__(self, devNum):
        self.cap = cv2.VideoCapture(devNum)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.frame = []

        # read standard of marker contour
        with np.load('circle.npz') as X:
            self.std_cnt_marker = [X[i] for i in X]

    def init(self):
        ret, frame = self.cap.read()
        self.frame = frame

    @staticmethod
    def px2m(px):
        m = K * px
        return m

    @staticmethod
    def m2px(px):
        px = m / K
        return px

    def get_mask(self):
        """
            Из изображении в динамическом цветовом диапазоне извлекаются
            маски заданных диапазонов.
        :return: mask
        """
        ret, frame = self.cap.read()
        self.frame = frame

        frame = cv2.blur(frame, (5, 5))
        hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)


        # red-front
        lower = np.array([140, 30, 149])
        upper = np.array([188, 141, 197])
        mask_front = cv2.inRange(hsv, lower, upper) # red

        # blue-back
        lower = np.array([92, 130, 136])
        upper = np.array([110, 242, 196])
        mask_rear = cv2.inRange(hsv, lower, upper)   # blue

        mask = mask_front + mask_rear


        #lower = np.array([0, 199, 49])
        #upper = np.array([164, 255, 255])
        #mask = cv2.inRange(hsv, lower, upper)


        cv2.imshow('mask', mask)
        return mask

    def get_markers(self):
        """
            1) Из self.get_mask() извлекается маска для нужных цветов;
            2) Из изображения вырезается все, чего нет в маске;
            3) Находятся все контуры
            4) Из контуров отсеивается все, что не соответствуйет
            заданной форме контура self.std_cnt_marker;
            5) Заполняется массив markers_objs
        :return: objects of class Marker
        """
        mask = self.get_mask()
        frame = self.frame.copy()
        img_markers = cv2.bitwise_and(frame, frame, mask=mask)

        __, cnts, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)   # for opencv 3.
        # cnts, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # matching circles. Don't use? becouse markers is tooooo smaaaall
        # for cnt_std in self.std_cnt_marker:
        # cnts_on_car_markers = []
        #     for cnt in cnts:
        #         ret = cv2.matchShapes(cnt, np.array(cnt_std, np.uint64), 1, 0)
        #         if ret < 1:
        #             cnts_on_car_markers.append(cnt)

        cnts_on_car_markers = cnts
        # cv2.drawContours(self.frame, cnts, -1, (220,220,0), 1)

        # creation Marker objects
        marker_objs = []
        for cnt in cnts_on_car_markers:
            # print(cv2.contourArea(cnt))
            if cv2.contourArea(cnt) > 25:
                M = cv2.moments(cnt)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                color = tuple(map(int, frame[cy, cx]))  # color in center of contour
                marker_objs.append(Marker(cx, cy, cnt, color))

        return marker_objs

    @staticmethod
    def draw_traj_real_time(blank_image, traj):
        if len(traj['x']) > 4:
            for i in range(3, len(traj['x'])):
                prev_x, prev_y = traj['x'][i-1], traj['y'][i-1]
                x, y = traj['x'][i], traj['y'][i]
                cv2.line(blank_image, (int(prev_x), int(prev_y)), (int(x), int(y)), (0,0,255), 1)
                # print(prev_x, prev_y)
        cv2.imshow('trajectory', blank_image)

    @staticmethod
    def draw_arrows(plt, frame, x, y, color, angle):
        for i in range(0, len(x), 5):
	    # print(angle[i] * 180 / np.pi)
	    theta = -angle[i] + np.pi/2
            plt.arrow(x[i], y[i], np.sin(theta) * 0.001, np.cos(theta) * 0.001,
                      head_width=0.03, head_length=0.05, color=color, width=0)

        plt.plot(x, y, color=color)

        plt.xlim([0, Camera.px2m(frame.shape[1])])
        plt.ylim([Camera.px2m(frame.shape[0]), 0])
