import numpy as np

from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

         
    
def f(x,u):
    x,u=x.flatten(),u.flatten()
    θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
    w_ap = array([[awind*cos(ψ-θ) - v],[awind*sin(ψ-θ)]])
    ψ_ap = angle(w_ap)
    a_ap=norm(w_ap)
    sigma = cos(ψ_ap) + cos(δsmax)
    if sigma < 0 :
        δs = pi + ψ_ap
    else :
        δs = -sign(sin(ψ_ap))*δsmax
    fr = p4*v*sin(δr)
    fs = p3*a_ap* sin(δs - ψ_ap)
    dx=v*cos(θ) + p0*awind*cos(ψ)
    dy=v*sin(θ) + p0*awind*sin(ψ)
    dv=(fs*sin(δs)-fr*sin(δr)-p1*v**2)/p8
    dw=(fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
    xdot=array([ [dx],[dy],[w],[dv],[dw]])
    return xdot,δs
    
p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,1,6000,1000,2000,1,1,2,300,10000
x = array([[10,-40,-3,1,0]]).T   #x=(x,y,θ,v,w)

dt = 0.1
awind,ψ = 2,-2
q = 0
a = array([[-50],[-100]])   
b = array([[50],[100]])
r = 10
ksi = pi/4
delta_r_max=1
beta = pi/4
                  
ax=init_figure(-100,100,-60,60)

def my_det(v_x,v_y):
    vx1,vx2 = v_x.flatten()
    vy1,vy2 = v_y.flatten()
    return vx1*vy2 - vy1*vx2

def controleur(x):
    global q
    m = array([[x[0,0]],[x[1,0]]])
    e = my_det((b-a)/np.linalg.norm(b-a),m-a)
    if abs(e)>r:
        q = np.sign(e)
    phi = np.arctan2((b-a)[1,0],(b-a)[0,0])
    theta_bar = phi - np.arctan2(e,r)
    if (np.cos(ψ - theta_bar) + np.cos(ksi)<0) or ((abs(e)-r<0) and (np.cos(phi - theta_bar) + np.cos(ksi)<0)):
        theta_bar = pi + ψ + q*ksi
    delta_r = (delta_r_max/pi)*sawtooth(x[2,0]-theta_bar)
    delta_s_max = (pi/2)*np.power(((cos(ψ-theta_bar)+1)/2),np.log10(pi/(2*beta))/np.log10(2))
    return array([[delta_r],[delta_s_max]])

for t in arange(0,60,0.1):
    clear(ax)
    plot([a[0,0],b[0,0]],[a[1,0],b[1,0]],'red')
    u=controleur(x)
    xdot,δs=f(x,u)
    x = x + dt*xdot
    print(x.shape)
    draw_sailboat(x,δs,u[0,0],ψ,awind)
    pause(0.01)

pause(10)

# singularite quand le bateau a le bon cap et est face au vent => immobile

