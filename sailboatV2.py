from numpy import *
from matplotlib.pyplot import *
from numpy.linalg import *

def sawtooth(x):
    return (x+pi)%(2*pi)-pi

def angle(x):
    x=x.flatten()
    return arctan2(x[1],x[0])

class sailboat:
    def __init__(self):
        # coeff
        self.p0 = 0.1
        self.p1 = 1
        self.p2 = 6000
        self.p3 = 1000
        self.p4 = 2000
        self.p5 = 1
        self.p6 = 1
        self.p7 = 2
        self.p8 = 300
        self.p9 = 10000
        # coordonnées
        self.x = array([[10, -40, -3, 1, 0]]).T  # x=(x,y,θ,v,w)
        self.dt = 0.1
        self.awind = 2
        self.phi = -2
        self.q = 0
        self.a = array([[-50], [-100]])
        self.b = array([[50], [100]])
        self.r = 10
        self.ksi = pi / 4
        self.delta_r_max = 1
        self.beta = pi / 4

    def f(self, u):
        self.x, u = self.x.flatten(), u.flatten()
        θ = self.x[2];
        v = self.x[3];
        w = self.x[4];
        δr = u[0];
        δsmax = u[1];
        w_ap = array([[self.awind * cos(self.phi - θ) - v], [self.awind * sin(self.phi - θ)]])
        phi_ap = angle(w_ap)
        a_ap = norm(w_ap)
        sigma = cos(phi_ap) + cos(δsmax)
        if sigma < 0:
            δs = pi + phi_ap
        else:
            δs = -sign(sin(phi_ap)) * δsmax
        fr = self.p4 * v * sin(δr)
        fs = self.p3 * a_ap * sin(δs - phi_ap)
        dx = v * cos(θ) + self.p0 * self.awind * cos(self.phi)
        dy = v * sin(θ) + self.p0 * self.awind * sin(self.phi)
        dv = (fs * sin(δs) - fr * sin(δr) - self.p1 * v ** 2) / self.p8
        dw = (fs * (self.p5 - self.p6 * cos(δs)) - self.p7 * fr * cos(δr) - self.p2 * w * v) / self.p9
        xdot = array([[dx], [dy], [w], [dv], [dw]])
        return xdot, δs

    def my_det(self,v_x, v_y):
        vx1, vx2 = v_x.flatten()
        vy1, vy2 = v_y.flatten()
        return vx1 * vy2 - vy1 * vx2

    def controleur(self):
        m = array([[self.x[0, 0]], [self.x[1, 0]]])
        e = sailboat.my_det(self,(self.b - self.a) / np.linalg.norm(self.b - self.a), m - self.a)
        if abs(e) > self.r:
            self.q = np.sign(e)
        phi = np.arctan2((self.b - self.a)[1, 0], (self.b - self.a)[0, 0])
        theta_bar = phi - np.arctan2(e, self.r)
        if (np.cos(self.phi - theta_bar) + np.cos(self.ksi) < 0) or ((abs(e) - self.r < 0) and (np.cos(phi - theta_bar) + np.cos(self.ksi) < 0)):
            theta_bar = pi + self.phi + self.q * self.ksi
        delta_r = (self.delta_r_max / pi) * sawtooth(self.x[2, 0] - theta_bar)
        delta_s_max = (pi / 2) * np.power(((cos(self.phi - theta_bar) + 1) / 2), np.log10(pi / (2 * self.beta)) / np.log10(2))
        return array([[delta_r], [delta_s_max]])