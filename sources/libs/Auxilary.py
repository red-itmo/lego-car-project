from math import pi


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

def multmat(X, Y): # X is matrice, Y is number
    if (type(X) is not list) and (type(Y) is not float or int):
        raise ValueError("Argument types are not compatible")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] * Y
        return result

def divmat(X, Y): # X is matrice, Y is number
    if (type(X) is not list) and (type(Y) is not float or int):
        raise ValueError("Argument types are not compatible")
    else:
        result = [[0 for i in range(len(X[0]))] for j in range(len(X))]
        for i in range(len(X)):
            for j in range(len(X[0])):
                result[i][j] += X[i][j] / Y
        return result
