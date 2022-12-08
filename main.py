import numpy as np
from numpy import *
from matplotlib.pyplot import *
from numpy.linalg import *
from sailboatV2 import *

boat = sailboat()
fig = figure()
ax = fig.add_subplot(111, aspect='equal')

for t in arange(0,1000,0.1):
    cla()
    ax.set_xlim(-200, 200)
    ax.set_ylim(-100, 100)
    plot([boat.a[0,0],boat.b[0,0]],[boat.a[1,0],boat.b[1,0]],'red')
    plot([boat.a[0,0],boat.c[0,0]],[boat.a[1,0],boat.c[1,0]],'blue')
    plot([boat.b[0,0],boat.c[0,0]],[boat.b[1,0],boat.c[1,0]],'green')

    u=boat.controleur()
    xdot,δs=boat.f(u)
    boat.x = (boat.x + (boat.dt*xdot).T).T

    #### Tracé du bâteau
    a,b,c = 5,7,2
    m = array([[0,c,c,-c,-c,0],[b,b-c,-a,-a,(b-c),b]])
    teta = (boat.x[2][0] + ((3*np.pi)/2))%(2*np.pi)
    bp = array([[np.cos(teta),-np.sin(teta)],[np.sin(teta),np.cos(teta)]])
    r = np.dot(bp,m)
    r += array([np.ones(6),np.zeros(6)])*boat.x[0]
    r += array([np.zeros(6),np.ones(6)])*boat.x[1]
    plot(r[0, :], r[1, :], color='red')

    #### Tracé du Vent
    f = array([[-10,10,8],[0,0,2]])
    phi = boat.phi
    bp_v = array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    r_v = np.dot(bp_v, f)
    r_v += array([np.ones(3), np.zeros(3)]) * (-80)
    r_v += array([np.zeros(3), np.ones(3)]) * 40
    plot(r_v[0, :], r_v[1, :], color='blue')
    f = array([[-10, 10, 8], [0, 0, -2]])
    phi = boat.phi
    bp_v = array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    r_v = np.dot(bp_v, f)
    r_v += array([np.ones(3), np.zeros(3)]) * (-80)
    r_v += array([np.zeros(3), np.ones(3)]) * 40
    plot(r_v[0, :], r_v[1, :], color='blue')



    pause(0.01)
pause(10)