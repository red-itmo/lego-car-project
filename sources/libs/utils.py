from libs.TrajectoryStuff import Point


def matmult(a, b):
    """Just multiply matrices a and b"""

    res = list([0, 0, 0, 0])
    for i in range(len(a)):
        for j in range(len(a)):
            res[i] += a[i][j] * b[j]
    return res


def mean(numbers):
    """Mean value of list of numbers"""
    return float(sum(numbers)) / max(len(numbers), 1)


def meanFilter(raw_data):
    """Mean filter"""
    well_data = []
    mean_data = mean([point.y for point in raw_data])
    for point in raw_data:
        if point.y > mean_data:
            well_data.append(Point(point.x, point.y))
    return well_data, mean_data
