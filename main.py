import numpy as np
from numpy import *
from matplotlib.pyplot import *
from numpy.linalg import *
from sailboatV2 import *

boat = sailboat()
fig = figure()
ax = fig.add_subplot(111, aspect='equal')


def color(n):
    r = [0.2627,0.6039,0.5921]
    g = [0.7960,0.9294,0.8352]
    c = np.ones((4,n))
    for i in range(n):
        for j in range(3):
            c[j,i] = round( r[j] + (g[j]-r[j])*(i/n),4)
    return c.T


colors = color(len(boat.objective))

dt,t=0.1,0
while boat.finish != len(boat.objective):

    cla()
    ax.set_xlim(-200, 200)
    ax.set_ylim(-100, 100)
    for index, objectif in enumerate(boat.objective):
        if index != len(boat.objective)-1:
            plot([boat.objective[index][0, 0],boat.objective[index+1][0, 0]], [boat.objective[index][1, 0], boat.objective[index+1][1, 0]], color=colors[index])
        else :
            plot([boat.objective[0][0, 0], boat.objective[index][0, 0]], [boat.objective[0][1, 0], boat.objective[index][1, 0]],color=colors[index])
        ax.plot(objectif[0], objectif[1], 'ro')

    u=boat.controleur()
    xdot,δs=boat.f(u)
    boat.x = (boat.x + (dt*xdot).T).T

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


    t+=dt #dt
    dt+=0.1
    pause(0.01)

print('temps de parcours : ', t)
pause(10)
