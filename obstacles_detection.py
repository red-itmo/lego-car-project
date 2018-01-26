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
		# Get all contours
		obst_cntrs = cv2.findContours( edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS )[-2]

		### TODO: FILTER CONTOURS IN ORDER TO GET RID OF SMALL NOISE AND STUFF ( LINE 47 )

		return obst_cntrs


"""while True:
	#ret, frame = camera.read()
	frame = cv2.imread("floor_with_obsticles.png")
	ret = True

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	blurred = cv2.bilateralFilter( gray, 12, 100, 100 )
	_, mask = cv2.threshold( blurred, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU )

	mask = cv2.erode(mask, None, iterations=3)

	edges = cv2.Canny( mask, 100, 200 )
	obst_cntrs = cv2.findContours( edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS )[-2]

	if obst_cntrs:
		for cntr in obst_cntrs:
			if cv2.contourArea( cntr ) > 450:
				obst_coord = cv2.minAreaRect( cntr )
				points = cv2.boxPoints( obst_coord )
				points = np.int0( np.around(points) )
				print points
				for i in range( -1, len(points) - 1 ):
					cv2.line(frame, tuple(points[i]), tuple(points[i+1]), (255, 0, 0), 3)

	if not ret:
		break

	if cv2.waitKey(1) & 0xFF is ord('q'):
		cv2.destroyAllWindows()
		break

	cv2.imshow("Frame", frame)
	cv2.imshow("Edges", edges)
	cv2.imshow("Mask", mask)

print obst_cntrs"""
