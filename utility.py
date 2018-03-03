import math

def areOn1Line(x1, y1, x2, y2, x3, y3):
	if (x2 - x1 == 0 or y2 - y1 == 0):
		return True
	return ((x3 - x1) / (x2 - x1) == (y3 - y1) / (y2 - y1))

def coord_to_angle(x1, y1, x2, y2):
	return math.atan2(y1 - y2, x1 - x2) / math.pi * 180

def rasst(x1, y1, x2, y2):
	return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def isInGoalCircle(point,goal):
	d = math.sqrt(math.pow((point.position[0]-goal[0]),2)+math.pow((point.position[1]-goal[1]),2))
	return d<20

def isInObstCircle(point,obs_point):
	d = math.sqrt(math.pow((point[0]-obs_point[0]),2)+math.pow((point[1]-obs_point[1]),2))
	return d<40

def onSegment(p,q,r):
	if(q[0]<=max(p[0],r[0]) and q[0]>=min(p[0],r[0])
			and q[1]<=max(p[1],r[1]) and q[1]>=min(p[1],r[1])):
		return True
	return False

def orientation(p,q,r):
	val = (q[1]-p[1])*(r[0]-q[0])-(q[0]-p[0])*(r[1]-q[1])

	if(val==0):
		return 0

	return 1 if val>0 else 2

def doIntersect(p_1_begin,p_1_end,p_2_begin,p_2_end):
	o1 = orientation(p_1_begin,p_1_end,p_2_begin)
	o2 = orientation(p_1_begin,p_1_end,p_2_end)
	o3 = orientation(p_2_begin,p_2_end,p_1_begin)
	o4 = orientation(p_2_begin,p_2_end,p_1_end)

	if(o1!=o2 and o3!=o4):
		return True

	if(o1==0 and onSegment(p_1_begin,p_2_begin,p_1_end)):
		return True

	if(o2==0 and onSegment(p_1_begin,p_2_begin,p_1_end)):
		return True

	if(o3==0 and onSegment(p_2_begin,p_1_begin,p_2_end)):
		return True

	if(o4==0 and onSegment(p_2_begin,p_1_end,p_2_end)):
		return True

	return False

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def CrossesObstacles(p_1,p_2,obstacles):
	CrossesObstacle = False
	for obstacle in obstacles:
		if(doIntersect(p_1,p_2,obstacle[0],obstacle[-1])):
			return True

def LiesBeneathObstacles(p,obstacles):

	for obstacle in obstacles:
		for point in obstacle:
			if(self.isInObstCircle(p,point)):
				return True

def inRangeOfImg(points,img):
	width,height = img.shape[:2]
	for point in points:
		if(point[1]>=width-5 or point[0]>=height-5 or point[1]<=10 or point[0]<=10):
			return False
	return True

def build_wall(begin_coord,end_coord,n_of_points):
	x_spacing = (end_coord[0]-begin_coord[0])/(n_of_points+1)
	y_spacing = (end_coord[1]-begin_coord[1])/(n_of_points+1)

	return [[int(begin_coord[0]+i*x_spacing),int(begin_coord[1]+i*y_spacing)]
				for i in range(1,n_of_points+1)]

def ClosestPoint(A,B,P):

	def find_perpendicular(A,B,P):
		a_to_p = [P.x - A.x,P.y-A.y]
		a_to_b = [B.x - A.x,B.y-A.y]

		atb2 = math.pow(a_to_b[0],2)+math.pow(a_to_b[1],2)

		atp_dot_atb = a_to_p[0]*a_to_b[0] + a_to_p[1]*a_to_b[1]

		t = sigmoid(atp_dot_atb/atb2)

		return [A.x+a_to_b[0]*t,A.y+a_to_b[1]*t]

	a_to_p = [P.x - A.x,P.y-A.y]
	a_to_b = [B.x - A.x,B.y-A.y]
	b_to_a = [A.x-B.x,A.y-B.x]
	b_to_p = [P.x-B.x,P.y-B.y]

	atb_dot_atp = dot_product(a_to_b,a_to_p)
	bta_dot_btp = dot_product(b_to_a,b_to_p)

	if(atb_dot_atp > 0):
		if(bta_dot_btp > 0):
			return find_perpendicular(A,B,P)
		else:
			return [B.x,B.y]
	else:
		return [A.x,A.y]

def dot_product(vector1,vector2):
	return vector1[0]*vector2[0]+vector1[1]*vector2[1]

def sigmoid(t):
	return 1/(1+math.exp(-t))
