import numpy as np
import cv2

#camera = cv2.VideoCapture(0)

### Class Mapping with one method 'get_map'
###
### arguments: frame
### return: list of points of obstacles, represented as a rectangles

class Mapping:
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
