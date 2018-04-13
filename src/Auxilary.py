from math import pi, sqrt, cos, sin, copysign, pow, atan2, exp
import numpy as np
from src.TrajectoryStuff_02 import Pose


class AngleFinder:
    def __init__(self):
        self.is_first_time = True
        self.true_angle = 0
        self.last_angle = 0

    def getAngle(self, bad_angle):
        if self.is_first_time:
            self.true_angle = bad_angle
            self.is_first_time = False
        else:
            if -pi < bad_angle - self.last_angle < pi:
                self.true_angle += bad_angle - self.last_angle
            elif pi < bad_angle - self.last_angle:
                self.true_angle += bad_angle - self.last_angle - 2 * pi
            else:
                self.true_angle += bad_angle - self.last_angle + 2 * pi

        self.last_angle = bad_angle

        return self.true_angle


########################################################################################################################

def matmult(X, Y):
    if len(X[0]) != len(Y):
        raise ValueError("Matrices size are not factorable")
    else:
        result = [[0 for i in range(len(Y[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(Y[0])):
                for k in range(len(Y)):
                    result[i][j] += X[i][k] * Y[k][j]
        return result


def matdiv(X, Y):
    if len(X[0]) != len(Y):
        raise ValueError("Matrices size are not factorable")
    else:
        result = [[0 for i in range(len(Y[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(Y[0])):
                for k in range(len(Y)):
                    result[i][j] += X[i][k] / Y[k][j]
        return result


def matdiff(X, Y):
    if (len(X) != len(Y)) or (len(X[0]) != len(Y[0])):
        raise ValueError("Matrices size are not subtractable")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] - Y[i][j]
        return result


def matadd(X, Y):
    if (len(X) != len(Y)) or (len(X[0]) != len(Y[0])):
        raise ValueError("Matrices size are not subtractable")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] + Y[i][j]
        return result


def multmat(X, Y):  # X is matrice, Y is number
    if (type(X) is not list) and (type(Y) is not float or int):
        raise ValueError("Argument types are not compatible")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] * Y
        return result


def divmat(X, Y):  # X is matrice, Y is number
    if (type(X) is not list) and (type(Y) is not float or int):
        raise ValueError("Argument types are not compatible")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] / Y
        return result


########################################################################################################################

def f(x):
    return (1 + 0.926 * x) / (2 + 1.792 * x + 3.104 * x ** 2)


def g(x):
    return 1 / (2 + 4.412 * x + 3.492 * x ** 2 + 6.67 * x ** 3)


def Cf(x):
    return 1 / 2 + f(x) * sin(pi / 2 * x ** 2) - g(x) * cos(pi / 2 * x ** 2)


def Sf(x):
    return 1 / 2 - f(x) * cos(pi / 2 * x ** 2) - g(x) * sin(pi / 2 * x ** 2)


def getClosest(x_r, y_r):
    pass


def xCoord(gamma, alpha, s):
    return gamma * sqrt(pi / abs(alpha)) * Cf(sqrt(abs(alpha) / pi) * s)


def yCoord(gamma, alpha, s):
    return gamma * copysign(1, alpha) * sqrt(pi / abs(alpha)) * Sf(sqrt(abs(alpha) / pi) * s)


def X(b):
    return copysign(1, b) * sqrt(pi * abs(b)) * Cf(sqrt(abs(b) / pi))


def Y(b):
    return sqrt(pi * abs(b)) * Sf(sqrt(abs(b) / pi))


def A(b1, b2=None):
    if b2 is None:
        return X(b1) * (1 + cos(b1)) + Y(b1) * sin(b1)
    else:
        return X(b1) * (1 + cos(b1 + b2)) + Y(b1) * sin(b1 + b2) + sin(0.5 * b1 + b2) - sin(0.5 * b1)


def B(b1, b2=None):
    if b2 is None:
        return X(b1) * sin(b1) + Y(b1) * (1 - cos(b1))
    else:
        return X(b1) * sin(b1 + b2) + Y(b1) * (1 - cos(b1 + b2)) - cos(0.5 * b1 + b2) + cos(0.5 * b1)


def C(b, psi):
    if len(b) == 1:
        return A(b[0]) * cos(psi) - B(b[0]) * sin(psi)
    else:
        return A(b[0], b[1]) * cos(psi) - B(b[0], b[1]) * sin(psi)


def D(b, psi):
    if len(b) == 1:
        return A(b[0]) * sin(psi) + B(b[0]) * cos(psi)
    else:
        return A(b[0], b[1]) * sin(psi) + B(b[0], b[1]) * cos(psi)


def G(b, psi):
    return B(b + psi) + D([b], psi)

########################################################################################################################

def calcClothoidPoints(gamma, alpha, s_end, type, step):
    poses = np.ndarray(shape=(int(s_end // step) + 2, 3))
    if (type == "in"):
        i = 0
        for s in np.arange(0, s_end, step):
            poses[i] = np.array([xCoord(gamma, alpha, s), yCoord(gamma, alpha, s), 0.5 * alpha * pow(s, 2)]).reshape((3))
            i += 1
        poses[i] = np.array([xCoord(gamma, alpha, s_end), yCoord(gamma, alpha, s_end), 0.5 * alpha * pow(s_end, 2)]).reshape((3))
    else:

        origin_pose = Pose(xCoord(-gamma, alpha, s_end), yCoord(-gamma, alpha, s_end), 0.5 * alpha * pow(s_end, 2))
        i = 0
        for s in np.arange(0, s_end, step):
            aux_pose = np.array([xCoord(-gamma, alpha, s_end - s), yCoord(-gamma, alpha, s_end - s),
                        0.5 * alpha * pow(s_end - s, 2)]).reshape((1, 3))
            poses[i] = transformCoords(origin_pose, aux_pose, 'b')
            i += 1
        aux_pose = np.array([xCoord(-gamma, alpha, 0.0), yCoord(-gamma, alpha, 0.0), 0.0]).reshape((1, 3))
        poses[i] = transformCoords(origin_pose, aux_pose, 'b')
    return poses


def calcArcPoints(point_c, point_0, point_1, step):
    k = 1 / point_c[0][1];
    delta = point_1[0][2];
    i = 0
    poses = np.ndarray(shape=(int(abs(delta) // abs(step*k)) + 2, 3))
    for d in np.arange(0, delta, copysign(1, delta)*abs(step*k)):
        poses[i] = [sin(d) / k, (1 - cos(d)) / k, d]
        i += 1
    poses[i] = [sin(delta) / k, (1 - cos(delta)) / k, delta]
    return poses


def calcStraightLinePoints(pose_0, pose_1, s_end, step):
    alpha = atan2(pose_1[0][1] - pose_0[0][1], pose_1[0][0] - pose_0[0][0])
    i = 0
    poses = np.ndarray(shape=(int(s_end // step) + 2, 3))
    for s in np.arange(0, s_end, step):
        poses[i] = [s * cos(alpha), s * sin(alpha), pose_0[0][2]]
        i += 1
    poses[i] = [s_end * cos(alpha), s_end * sin(alpha), pose_0[0][2]]
    return poses


def transformCoords(origin_pose=None, old_poses=None, direction=None):
    T = np.array([[cos(origin_pose.angle), -sin(origin_pose.angle), origin_pose.point.x],
         [sin(origin_pose.angle), cos(origin_pose.angle), origin_pose.point.y],
         [0, 0, 1]])
    new_poses = np.ndarray(shape=(old_poses.shape[0], 3))
    if direction == "b":
        T = np.linalg.inv(T)
        new_poses[:, 2] = old_poses[:, 2] - origin_pose.angle
    else:
        new_poses[:, 2] = old_poses[:, 2] + origin_pose.angle

    aux_poses = T.dot(np.vstack([old_poses[:, 0:2].T, (np.ones(shape=(old_poses[:,0].T.shape)))]))
    new_poses[:, 0:2] = aux_poses[0:2, :].T

    return new_poses


def calcPathElementPoints(element, start_pose, step):
    if element[0] == "ClothoidLine":
        aux_poses = calcClothoidPoints(element[2], element[3], element[4], element[5], step)
    elif element[0] == "StraightLine":
        aux_poses = calcStraightLinePoints(element[1], element[2], element[3], step)
    elif element[0] == "CircleLine":
        aux_poses = calcArcPoints(element[1], element[2], element[3], step)

    return transformCoords(start_pose, aux_poses)

########################################################################################################################

def areOn1Line(x1, y1, x2, y2, x3, y3):
	if (x2 - x1 == 0 or y2 - y1 == 0):
		return True
	return ((x3 - x1) / (x2 - x1) == (y3 - y1) / (y2 - y1))


def coord_to_angle(x1, y1, x2, y2):
	return atan2(y1 - y2, x1 - x2) / pi * 180


def rasst(x1, y1, x2, y2):
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


def isInGoalCircle(point,goal):
	d = sqrt(pow((point.position[0]-goal[0]), 2)+pow((point.position[1]-goal[1]), 2))
	return d < 20


def isInObstCircle(point,obs_point):
	d = sqrt(pow((point[0]-obs_point[0]), 2)+pow((point[1]-obs_point[1]), 2))
	return d < 40


def onSegment(p,q,r):
	if(q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0])
			and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
		return True
	return False


def orientation(p,q,r):
	val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - sq[1])

	if(val == 0):
		return 0

	return 1 if val > 0 else 2


def doIntersect(p_1_begin,p_1_end,p_2_begin,p_2_end):
	o1 = orientation(p_1_begin, p_1_end, p_2_begin)
	o2 = orientation(p_1_begin, p_1_end, p_2_end)
	o3 = orientation(p_2_begin, p_2_end, p_1_begin)
	o4 = orientation(p_2_begin, p_2_end, p_1_end)

	if(o1 != o2 and o3 != o4):
		return True

	if(o1==0 and onSegment(p_1_begin, p_2_begin, p_1_end)):
		return True

	if(o2==0 and onSegment(p_1_begin, p_2_begin, p_1_end)):
		return True

	if(o3==0 and onSegment(p_2_begin, p_1_begin, p_2_end)):
		return True

	if(o4==0 and onSegment(p_2_begin, p_1_end, p_2_end)):
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
		if(doIntersect(p_1, p_2, obstacle[0], obstacle[-1])):
			return True


def LiesBeneathObstacles(p,obstacles):
	for obstacle in obstacles:
		for point in obstacle:
			if(self.isInObstCircle(p, point)):
				return True


def inRangeOfImg(points,img):
	width,height = img.shape[:2]
	for point in points:
		if(point[1] >= width-5 or point[0] >= height-5 or point[1] <= 10 or point[0] <= 10):
			return False
	return True


def build_wall(begin_coord,end_coord,n_of_points):
	x_spacing = (end_coord[0] - begin_coord[0]) / (n_of_points + 1)
	y_spacing = (end_coord[1] - begin_coord[1]) / (n_of_points + 1)

	return [[int(begin_coord[0] + i*x_spacing),int(begin_coord[1] + i*y_spacing)]
				for i in range(1,n_of_points+1)]


def ClosestPoint(A,B,P):

	def find_perpendicular(A,B,P):
		a_to_p = [P.x - A.x, P.y - A.y]
		a_to_b = [B.x - A.x, B.y - A.y]

		atb2 = pow(a_to_b[0], 2) + pow(a_to_b[1], 2)

		atp_dot_atb = a_to_p[0]*a_to_b[0] + a_to_p[1]*a_to_b[1]

		t = sigmoid(atp_dot_atb/atb2)

		return [A.x + a_to_b[0]*t,A.y + a_to_b[1]*t]

	a_to_p = [P.x - A.x,P.y - A.y]
	a_to_b = [B.x - A.x,B.y - A.y]
	b_to_a = [A.x - B.x,A.y - B.x]
	b_to_p = [P.x - B.x,P.y - B.y]

	atb_dot_atp = dot_product(a_to_b, a_to_p)
	bta_dot_btp = dot_product(b_to_a, b_to_p)

	if(atb_dot_atp > 0):
		if(bta_dot_btp > 0):
			return find_perpendicular(A, B, P)
		else:
			return [B.x, B.y]
	else:
		return [A.x, A.y]


def build_wall(begin_coord,end_coord,n_of_points):
	x_spacing = (end_coord[0] - begin_coord[0]) / (n_of_points + 1)
	y_spacing = (end_coord[1] - begin_coord[1]) / (n_of_points + 1)

	return [[int(begin_coord[0] + i*x_spacing),int(begin_coord[1] + i*y_spacing)] for i in range(1, n_of_points+1)]




def dot_product(vector1,vector2):
	return vector1[0]*vector2[0] + vector1[1]*vector2[1]



def sigmoid(t):
	return 1 / (1 + exp(-t))
