from math import atan2, cos, sin, pi
from libs.Auxilary import AngleFinder

class PathController:# FIXME Class does not work with backwards directions or negative speed
    def __init__(self):
        self.k_theta = 20
        self.k_e = -50
        self.angle_finder = AngleFinder()

    def getControls(self, v_r, x_r, y_r, x_ref, y_ref, theta, kappa, t_hat):
        d = (x_r - x_ref, y_r - y_ref)
        e = d[0] * t_hat[1] - d[1] * t_hat[0]
        aux_angle = self.angle_finder.getAngle(atan2(t_hat[1], t_hat[0]))
        theta_e = theta - aux_angle + pi
        w = -(self.k_theta * abs(v_r) * theta_e)

        if (1 - kappa * e) != 0:
            w += (v_r * kappa * cos(theta_e) / (1 - kappa * e))
        if theta_e != 0:
            w -= (v_r * (self.k_e * sin(theta_e) / theta_e)) * e
        else:
            w -= (v_r * (self.k_e)) * e
        print("%f %f %f %f %f" % (d[0], d[1], e, aux_angle, w))
        return w