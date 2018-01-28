import cv2
import numpy as np



class Mapping:
# Constructor
	def __init__(self):
		# define the lower and upper boundaries of the colors of the markers
		# ball in the HSV color space
		self.blueLower = (96, 151, 138)
		self.blueUpper = (115, 255, 210)
		self.yellowLower = (0, 128, 108)
		self.yellowUpper = (38, 173, 255)

		# method gets tuples with x and y coordinates of blue and yellow centers
	def get_pose(self, blue, yellow):
			# for calculating the angle of a robot scalar multiplication of vectors is used
			orient_vector = ( yellow[0] - blue[0], yellow[1] - blue[1] )
			thetha = np.arccos( 1.0 * orient_vector[0] / np.sqrt( orient_vector[0] ** 2 + orient_vector[1] ** 2 ) )
			if blue[1] > yellow[1]:
				thetha = -thetha
			return ( int(blue[0]), int(blue[1]) ), thetha
		
		# TO GET THE POSE OF THE CAR, LAUNCH THIS FUNCTION
		# method gets a frame as an argument, returns tuple of coordinates of 
		# blue marker and angle of a robot
	def find_car(self, frame):
		# resize the frame, blur it, and convert it to the HSV
		# color space

		#frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (15, 15), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	 
		# construct a mask for the color blue, then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask_blue = cv2.inRange(hsv, self.blueLower, self.blueUpper)
		mask_blue = cv2.erode(mask_blue, None, iterations=2)
		mask_blue = cv2.dilate(mask_blue, None, iterations=2)


		blue_cntr = None
		blue_cntr = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]


		# for yellow
		mask_yellow = cv2.inRange(hsv, self.yellowLower, self.yellowUpper)
		mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
		mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
		
		yellow_cntr = None
		yellow_cntr = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]

		# only proceed if two contours were found
		if blue_cntr and yellow_cntr:

			# use founded contours to compute the minimum enclosing circles and
			# centroids
			(blue_coord, radius_blue) = cv2.minEnclosingCircle(blue_cntr[0])
			blue_coord = ( int(blue_coord[0]), int(blue_coord[1]) )
			M_blue = cv2.moments(blue_cntr[0])
			center_blue = (int(M_blue["m10"] / M_blue["m00"]), int(M_blue["m01"] / M_blue["m00"]))

			(yellow_coord, radius_yellow) = cv2.minEnclosingCircle(yellow_cntr[0])
			yellow_coord = ( int(yellow_coord[0]), int(yellow_coord[1]) )
			M_yellow = cv2.moments(yellow_cntr[0])
			center_yellow = (int(M_yellow["m10"] / M_yellow["m00"]), int(M_yellow["m01"] / M_yellow["m00"]))

			coords, angle = self.get_pose( blue_coord, yellow_coord )
			return coords, angle

		# if less than two contours found, return ( False, False )
		else:
			return ( False, False )


	
	# Method returns list of line coordinates of the obstacles
	def get_map(self, frame):
		# Convert image into grayscale
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		# Blur the image, apply binominal otsu filter
		blurred = cv2.bilateralFilter( gray, 12, 100, 100 )
		_, mask = cv2.threshold( blurred, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU )
		# Erode mask to avoid sharp edges
		mask = cv2.erode(mask, None, iterations=3)
		# Get mask of the image with founded edges
		edges = cv2.Canny( mask, 100, 200 )
		# Get all counters
		obst_cntrs = cv2.findContours( edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS )[-2]

		# Following loop deletes small contours
		obstacles = []
		for o in obst_cntrs:
			if cv2.contourArea(o) > 500:
				addToList = True
				obst_coord = cv2.minAreaRect( o )
				obst_coord = cv2.boxPoints( obst_coord )
				obst_coord = np.int0( np.around(obst_coord) )
				first_point = [ obst_coord[0] ]
				obst_coord = np.concatenate( (obst_coord, first_point ) )
				line_coord = []
				for i in range( 0, len(obst_coord) - 1 ):
					line_coord.append( [ obst_coord[i], obst_coord[i+1] ] )
				obstacles.append( line_coord )



		return obstacles
