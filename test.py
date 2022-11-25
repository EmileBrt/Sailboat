import matplotlib.pyplot as plt
import numpy as np
from numpy import *
teta = np.pi/6
a,b,c = 5,7,2

bp = array([[np.cos(teta),-np.sin(teta)],[np.sin(teta),np.cos(teta)]])
m = array([[0,c,c,-c,-c,0],[b,b-c,-a,-a,(b-c),b]])
print(bp.shape)
r = np.dot(bp,m)
print(r)

plt.plot(m[0,:],m[1,:], color='green')
plt.plot(r[0,:],r[1,:], color='red')
plt.show()