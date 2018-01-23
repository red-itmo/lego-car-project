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