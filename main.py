from numpy import *
from matplotlib.pyplot import *
from numpy.linalg import *
from sailboatV2 import *

boat = sailboat()
fig = figure()
ax = fig.add_subplot(111, aspect='equal')

for t in arange(0,60,0.1):
    cla()
    ax.set_xlim(-100, 100)
    ax.set_ylim(-60, 60)
    plot([boat.a[0,0],boat.b[0,0]],[boat.a[1,0],boat.b[1,0]],'red')
    u=boat.controleur()
    xdot,δs=boat.f(u)
    boat.x = (boat.x + (boat.dt*xdot).T).T
    plot(boat.x[0], boat.x[1], color='green', marker='o', linestyle='dashed',linewidth=2, markersize=6) #### à refaire
    pause(0.01)
pause(10)