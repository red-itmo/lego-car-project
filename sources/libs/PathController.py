from math import atan2, cos, sin

class PathController:
    def __init__(self):
        self.k_theta = 20
        self.k_e = 1

    def getControls(self, v_r, x_r, y_r, x_ref, y_ref, theta, kappa, t_hat):
        d = (x_r - x_ref, y_r - y_ref)
        e = d[0] * t_hat[1] - d[1] * t_hat[0]
        theta_e = theta - atan2(t_hat[1], t_hat[0])
        w = -(self.k_theta * abs(v_r) * theta_e)

        if (1 - kappa * e) != 0:
            w += (v_r * kappa * cos(theta_e) / (1 - kappa * e))
        if theta_e != 0:
            w -= (v_r * (self.k_e * sin(theta_e) / theta_e)) * e
        print("%f %f %f %f %f" % (d[0], d[1], e, theta_e, w))
        return w