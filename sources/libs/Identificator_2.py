import numpy as np


class Identificator:
    def __init__(self):
        self.last_theta = np.array([[1], [0]])
        self.last_p = np.array([[10, 0], [0, 10]])
        self.L = 0.165
        self.file = open("identificator.txt", "w")

    def update(self, v, omega, phi):
        xi = np.array([[v / self.L * phi], [v / self.L]])
        # print("XI = ", xi)
        xi_T = np.array([[v / self.L * phi, v / self.L]])
        # print("XI_T = ", xi_T)
        omega_hat = np.dot(xi_T,  self.last_theta)
        # print("omega_hat = ", omega_hat)
        eps = omega - omega_hat[0][0]
        # print("eps = ", eps)
        p = np.subtract(self.last_p, np.divide(np.dot(np.dot(np.dot(self.last_p, xi), xi_T), self.last_p), (1 + np.dot(np.dot(xi_T, self.last_p), xi)[0][0])))
        # print("p = ", p)
        theta = np.add(self.last_theta, np.dot(np.dot(p, xi), eps))
        # print("theta = ", theta)

        self.last_p = p
        self.last_theta = theta
        self.file.write("%f %f %f %f %f %f %f\n" % (theta[0][0], theta[1][0], eps, omega_hat[0][0], v, omega, phi))

        return self.last_theta

    def __delete__(self):
        self.file.close()


if __name__ == "__main__":
    id = Identificator()
    theta = id.update(5, 7, 0.5)
    print(theta)
