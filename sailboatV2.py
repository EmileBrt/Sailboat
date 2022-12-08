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
        # coefficients
        self.p0 = 0.1  # drift coefficient
        self.p1 = 1  # drag coefficient
        self.p2 = 6000  # angular friction of the hull against the water
        self.p3 = 1000  # sail lift
        self.p4 = 2000  # rudder lift
        self.p5 = 1  # position of the wind's center of thrust on the sail
        self.p6 = 1  # position of the mast
        self.p7 = 2  # position of the rudder
        self.p8 = 300  # mass of the sailboat
        self.p9 = 10000  # inertial momentum of the sailboat
        # coordonnées
        self.x = array([[10, -40, -3, 1, 0]]).T  # x=(x,y,θ,v,w)
        self.dt = 0.1
        self.awind = 2
        self.phi = -2
        self.q = 0
        # points qui donnent le cap à tenir
        self.a = array([[-50], [0]])  # pt a
        self.b = array([[50], [50]])  # pt b
        self.c = array([[100], [-50]])  # pt c
        self.pos = array([[self.x[0, 0]], [self.x[1, 0]]])
        self.objective = array([self.a,self.b,self.c])
        self.finish = 0

        self.r = 10  # couloir autour de la ligne à tenir
        self.ksi = pi / 4
        self.delta_r_max = 1
        self.beta = pi / 4  # coefficient obtenu avec l'équation résolue

    def f(self, u):
        """
        donne la dérivée en fonction du modèle : x' en fonction de u
        """
        self.pos = array([[self.x[0, 0]], [self.x[1, 0]]])
        self.x, u = self.x.flatten(), u.flatten()
        θ = self.x[2] # angle du voilier
        v = self.x[3] # vitesse du voilier
        w = self.x[4] # vitesse de rotation instantannée
        δr = u[0] # angle du gouvernail
        δsmax = u[1] # angle de la voile
        w_ap = array([[self.awind * cos(self.phi - θ) - v], [self.awind * sin(self.phi - θ)]])  # vent apparent, cf formule du poly
        phi_ap = angle(w_ap)   # direction du vent apparent
        a_ap = norm(w_ap)
        sigma = cos(phi_ap) + cos(δsmax)  # indicateur de la tension --> donne le comportement de la voile (tendue ou non)
        if sigma < 0:
            δs = pi + phi_ap # la voile n'est pas tendue donc se comporte comme un drapeau et va dans le sens du vent
        else:
            δs = -sign(sin(phi_ap)) * δsmax
        fr = self.p4 * v * sin(δr)
        fs = self.p3 * a_ap * sin(δs - phi_ap)
        dx = v * cos(θ) + self.p0 * self.awind * cos(self.phi)
        dy = v * sin(θ) + self.p0 * self.awind * sin(self.phi)
        dv = (fs * sin(δs) - fr * sin(δr) - self.p1 * v ** 2) / self.p8
        dw = (fs * (self.p5 - self.p6 * cos(δs)) - self.p7 * fr * cos(δr) - self.p2 * w * v) / self.p9
        xdot = array([[dx], [dy], [w], [dv], [dw]])
        return xdot, δs  # δs angle de la voile

    def rotationObjectif(self):
        self.objective = [self.objective[2], self.objective[0], self.objective[1]]
        self.finish = self.finish + 1


    #fonction qui calcule une distance entre deux points
    def distance(self, point):
        return sqrt((self.x[0, 0] - point[0,0]) ** 2 + (self.x[1, 0] - point[1,0]) ** 2)

    def my_det(self,v_x, v_y):
        """

        """
        vx1, vx2 = v_x.flatten()
        vy1, vy2 = v_y.flatten()
        return vx1 * vy2 - vy1 * vx2

    def controleur(self):

        #le cap à suivre le cap par défaut est [ac]
        if self.distance(self.objective[2]) <= 5:
            self.rotationObjectif()
            print('objectif changé', self.objective)
            print('nouvelle distance = ', self.distance(self.objective[2]))

        m = array([[self.x[0, 0]], [self.x[1, 0]]])
        e = sailboat.my_det(self,(self.objective[2] - self.objective[0]) / np.linalg.norm(self.objective[2] - self.objective[0]), m - self.objective[0])
        if abs(e) > self.r:
            self.q = np.sign(e)
        phi = np.arctan2((self.objective[2] - self.objective[0])[1, 0], (self.objective[2] - self.objective[0])[0, 0])
        theta_bar = phi - np.arctan2(e, self.r)
        if (np.cos(self.phi - theta_bar) + np.cos(self.ksi) < 0) or ((abs(e) - self.r < 0) and (np.cos(phi - theta_bar) + np.cos(self.ksi) < 0)):
            theta_bar = pi + self.phi - self.q * self.ksi
        delta_r = (self.delta_r_max / pi) * sawtooth(self.x[2, 0] - theta_bar)
        delta_s_max = (pi / 2) * np.power(((cos(self.phi - theta_bar) + 1) / 2), np.log10(pi / (2 * self.beta)) / np.log10(2))
        return array([[delta_r], [delta_s_max]])

