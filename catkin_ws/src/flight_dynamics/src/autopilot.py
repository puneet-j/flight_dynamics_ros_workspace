#!/usr/bin/python


import rospy
import numpy as np
from std_msgs.msg import Float32
from tf.broadcaster import TransformBroadcaster
import tf
from tf.transformations import rotation_matrix, concatenate_matrices, euler_from_matrix
# import time
from geometry_msgs.msg import Twist
from scipy import signal
import scipy




#sys1 = signal.TransferFunction(num1,den1)
t0 = 0.0
V_a = 0.1
inp_ = [[0.00],[0.00],[0.00]]
wind = [[0.00],[0.00],[0.00]]
inp = [[0.0],[0.],[0.0],[0.0]]#,[0.],[0.]]
# state = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
state = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
gusts = [[0.0],[0.0],[0.0]]


beta = 0.0
alpha = 0.0


del_a_max = 3.14/3.0
# e_phi_max = 3.14/6.0
# zeta_phi = 0.8
#
# W_course = 10.0
# zeta_course = 0.8
#
# del_t_max = 10.0
# del_e_max = 3.14/3.0
# e_theta_max = 3.14/6.0
# zeta_theta = 0.8
# W_h = 8.0
# zeta_h = 0.8
# W_va = 7.0
# zeta_va = 0.8
# omega_t = 2.0
# zeta_t = 0.8
#


def start_funct():
    global param,time_old,inp, state, V_a, abp
    # initialize member variables
    # print "do i get here?"
    # get parameters
    try:
        param_namespace = '/flight_dynamics'
        param = rospy.get_param(param_namespace)
    except KeyError:
        rospy.logfatal('Parameters not set in ~/flight_dynamics namespace')
        rospy.signal_shutdown('Parameters not set')
    # print "hi i am before subscribers"
    # publish/subscribe:
    V_a = 17.0
    #R_ = 10000000000.0
    #gamma_ = 0.0
    # print "hi"

    #x_opt = trim(V_a,R_, gamma_)
    #abp = x_opt.x
    #get_inp(abp,V_a,R_,gamma_)
    #print inp

    rospy.Subscriber("/flight_dynamics/h", Float32, update_h)
    rospy.Subscriber("/flight_dynamics/va", Float32, update_va)
    rospy.Subscriber("/flight_dynamics/course", Float32, update_course)
    #rospy.Subscriber("/flight_dynamics/del_t", Float32, update_del_t)

    #rospy.Subscriber("/flight_dynamics/w_n", Float32, update_w_n)
    #rospy.Subscriber("/flight_dynamics/w_e", Float32, update_w_e)
    #rospy.Subscriber("/flight_dynamics/w_d", Float32, update_w_d)

    time_old = rospy.Time.now().to_sec()

    dynamics_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/dynamics_rate), dynamics_timer_callback)
    rospy.spin()

# def get_inp(a,V_a,R,gamma):
#     global inp, param, state
#
#     g = 9.81
#     m = param['m']
#     b = param['b']
#     c = param['c']
#     S = param['S']
#     rho = param['rho']
#     Jx = param['Jx']
#     Jy = param['Jy']
#     Jz = param['Jz']
#     Jxz = param['Jxz']
#     k_motor = param['k_motor']
#     alpha,beta,phi = a
#     # print a,alpha,beta,phi
#     u = V_a*np.cos(alpha)*np.cos(beta)
#     v = V_a*np.sin(beta)
#     w = V_a*np.sin(alpha)*np.cos(beta)
#
#     theta = alpha + gamma
#     p = -V_a*np.sin(theta)/R
#     q = V_a*np.sin(phi)*np.cos(theta)/R
#     r = V_a*np.cos(phi)*np.cos(theta)/R
#     s = np.copy(state)
#     s[6][0] = phi
#     s[3][0] = u
#     s[4][0] = v
#     s[5][0] = w
#     s[7][0] = theta
#     s[9][0] = p
#     s[10][0] = q
#     s[11][0] = r
#     state = np.copy(s)
#     # print state, "trim state"
#     sphi   = np.sin(phi)
#     cphi   = np.cos(phi)
#     stheta = np.sin(theta)
#     ctheta = np.cos(theta)
#     ttheta = np.tan(theta)
#     secth = 1.0/ctheta
#     calpha = np.cos(alpha)
#     salpha = np.sin(alpha)
#
#     C_D_alpha = param['C_D_0'] + param['C_D_alpha']*alpha
#     C_L_alpha = param['C_L_0'] + param['C_L_alpha']*alpha
#     C_X = -C_D_alpha*calpha + C_L_alpha*salpha
#     C_X_q = -param['C_D_q']*calpha + param['C_L_q']*salpha
#     C_X_delta_e = -param['C_D_delta_e']*calpha + param['C_L_delta_e']*salpha
#     C_Z = -C_D_alpha*salpha - C_L_alpha*calpha
#     C_Z_q = -param['C_D_q']*salpha - param['C_L_q']*calpha
#     C_Z_delta_e = -param['C_D_delta_e']*salpha - param['C_L_delta_e']*calpha
#
#     Gamma = Jx*Jz - Jxz**2
#     Gamma_1 = Jxz*(Jx - Jy + Jz)/Gamma
#     Gamma_2 = (Jz*(Jz-Jy) + Jxz**2)/Gamma
#     Gamma_3 = Jz/Gamma
#     Gamma_4 = Jxz/Gamma
#     Gamma_7 = ((Jx - Jy)*Jx + Jxz**2)/Gamma
#     Gamma_8 = Jx/Gamma
#
#     C_p_delta_a = Gamma_3*param['C_l_delta_a'] + Gamma_4*param['C_n_delta_a']
#     C_p_delta_r = Gamma_3*param['C_l_delta_r'] + Gamma_4*param['C_n_delta_r']
#     C_r_delta_a = Gamma_4*param['C_l_delta_a'] + Gamma_8*param['C_n_delta_a']
#     C_r_delta_r = Gamma_4*param['C_l_delta_r'] + Gamma_8*param['C_n_delta_r']
#     C_p_0 = Gamma_3*param['C_l_0'] + Gamma_4*param['C_n_0']
#     C_p_beta = Gamma_3*param['C_l_beta'] + Gamma_4*param['C_n_beta']
#     C_p_p = Gamma_3*param['C_l_p'] + Gamma_4*param['C_n_p']
#     C_p_r = Gamma_3*param['C_l_r'] + Gamma_4*param['C_n_r']
#     C_r_0 = Gamma_4*param['C_l_0'] + Gamma_8*param['C_n_0']
#     C_r_beta = Gamma_4*param['C_l_beta'] + Gamma_8*param['C_n_beta']
#     C_r_p = Gamma_4*param['C_l_p'] + Gamma_8*param['C_n_p']
#     C_r_r = Gamma_4*param['C_l_r'] + Gamma_8*param['C_n_r']
#
#     del_e = (((Jxz*(p**2.0 - r**2.0) + (Jx - Jz)*p*r)/(0.5*rho*(V_a**2.0)*c*S)) - \
#             param['C_m_0'] - param['C_m_alpha']*alpha - param['C_m_q']*c*q/(2.0*V_a))/param['C_m_delta_e']
#     del_t = np.sqrt(((2.0*m*(-r*v + q*w + g*stheta))-\
#             (rho*(V_a**2.0)*S*(C_X + C_X_q*c*q/(2.0*V_a) +  C_X_delta_e*del_e)))\
#             /(rho*param['S_prop']*param['C_prop']*(k_motor**2.0))\
#             +(V_a**2.0)/(k_motor**2.0))
#     dels = np.linalg.inv(np.matrix([[C_p_delta_a, C_p_delta_r],[C_r_delta_a, C_r_delta_r]]))*\
#            np.matrix([[((-Gamma_1*p*q + Gamma_2*q*r)/(0.5*rho*(V_a**2)*S*b)) - C_p_0 - C_p_beta*beta - C_p_p*b*p/(2.0*V_a) - C_p_r*b*r/(2.0*V_a)]\
#            ,[((-Gamma_7*p*q + Gamma_1*q*r)/(0.5*rho*(V_a**2)*S*b)) - C_r_0 - C_r_beta*beta - C_r_p*b*p/(2.0*V_a) - C_r_r*b*r/(2.0*V_a)]])
#
#     del_a = dels[0,0]
#     del_r = dels[1,0]
#
#     inp = [del_a,del_e,del_r,del_t]
#
#
# def f(A,*args):
#     global param, state
#     g = 9.81
#     m= param['m']
#     b = param['b']
#     c = param['c']
#     S = param['S']
#     rho = param['rho']
#     Jx = param['Jx']
#     Jy = param['Jy']
#     Jz = param['Jz']
#     Jxz = param['Jxz']
#     k_motor = param['k_motor']
#     # print "f start"
#     alpha,beta,phi = A
#     V_a,gamma,x_,R = args
#
#     theta = alpha + gamma
#
#     calpha = np.cos(alpha)
#     salpha = np.sin(alpha)
#     cbeta = np.cos(beta)
#     sbeta = np.sin(beta)
#     sphi   = np.sin(phi)
#     cphi   = np.cos(phi)
#     stheta = np.sin(theta)
#     ctheta = np.cos(theta)
#     ttheta = np.tan(theta)
#     secth = 1.0/ctheta
#
#     u = V_a*calpha*cbeta
#     v = V_a*sbeta
#     w = V_a*salpha*cbeta
#
#
#     p = -V_a*stheta/R
#     q = V_a*sphi*ctheta/R
#     r = V_a*cphi*ctheta/R
#     #
#
#
#     C_D_alpha = param['C_D_0'] + param['C_D_alpha']*alpha
#     C_L_alpha = param['C_L_0'] + param['C_L_alpha']*alpha
#     C_X = -C_D_alpha*calpha + C_L_alpha*salpha
#     C_X_q = -param['C_D_q']*calpha + param['C_L_q']*salpha
#     C_X_delta_e = -param['C_D_delta_e']*calpha + param['C_L_delta_e']*salpha
#     C_Z = -C_D_alpha*salpha - C_L_alpha*calpha
#     C_Z_q = -param['C_D_q']*salpha - param['C_L_q']*calpha
#     C_Z_delta_e = -param['C_D_delta_e']*salpha - param['C_L_delta_e']*calpha
#
#     Gamma = Jx*Jz - Jxz**2
#     Gamma_1 = Jxz*(Jx - Jy + Jz)/Gamma
#     Gamma_2 = (Jz*(Jz-Jy) + Jxz**2)/Gamma
#     Gamma_3 = Jz/Gamma
#     Gamma_4 = Jxz/Gamma
#     Gamma_7 = ((Jx - Jy)*Jx + Jxz**2)/Gamma
#     Gamma_8 = Jx/Gamma
#
#     C_p_delta_a = Gamma_3*param['C_l_delta_a'] + Gamma_4*param['C_n_delta_a']
#     C_p_delta_r = Gamma_3*param['C_l_delta_r'] + Gamma_4*param['C_n_delta_r']
#     C_r_delta_a = Gamma_4*param['C_l_delta_a'] + Gamma_8*param['C_n_delta_a']
#     C_r_delta_r = Gamma_4*param['C_l_delta_r'] + Gamma_8*param['C_n_delta_r']
#     C_p_0 = Gamma_3*param['C_l_0'] + Gamma_4*param['C_n_0']
#     C_p_beta = Gamma_3*param['C_l_beta'] + Gamma_4*param['C_n_beta']
#     C_p_p = Gamma_3*param['C_l_p'] + Gamma_4*param['C_n_p']
#     C_p_r = Gamma_3*param['C_l_r'] + Gamma_4*param['C_n_r']
#     C_r_0 = Gamma_4*param['C_l_0'] + Gamma_8*param['C_n_0']
#     C_r_beta = Gamma_4*param['C_l_beta'] + Gamma_8*param['C_n_beta']
#     C_r_p = Gamma_4*param['C_l_p'] + Gamma_8*param['C_n_p']
#     C_r_r = Gamma_4*param['C_l_r'] + Gamma_8*param['C_n_r']
#     del_e = (((Jxz*(p**2.0 - r**2.0) + (Jx - Jz)*p*r)/(0.5*rho*(V_a**2.0)*c*S)) - \
#             param['C_m_0'] - param['C_m_alpha']*alpha - (param['C_m_q']*c*q/(2.0*V_a)))/param['C_m_delta_e']
#     # print ((V_a**2.0)/(k_motor**2.0)),m,b,c,p,q,r,w,g,stheta,rho,C_X,C_X_q,C_X_delta_e,del_e,"all variables"
#
#     # print "part by part del_t break down", (2*m*(-r*v + q*w + g*stheta)),(rho*(V_a**2.0)*S*(C_X + C_X_q*c*q/(2*V_a) +  C_X_delta_e*del_e)), \
#     #         (rho*param['S_prop']*param['C_prop']*(k_motor**2.0)), (V_a**2.0)/(k_motor**2.0)
#     # print "why is this so big", (((2*m*(-r*v + q*w + g*stheta))-\
#     #         (rho*(V_a**2.0)*S*(C_X + (C_X_q*c*q/(2*V_a)) +  (C_X_delta_e*del_e))))\
#     #         /(rho*param['S_prop']*param['C_prop']*(k_motor**2.0)))
#     #
#     del_t = np.sqrt((((2*m*(-r*v + q*w + g*stheta))-\
#             (rho*(V_a**2.0)*S*(C_X + (C_X_q*c*q/(2*V_a)) +  (C_X_delta_e*del_e))))\
#             /(rho*param['S_prop']*param['C_prop']*(k_motor**2.0)))+((V_a**2.0)/(k_motor**2.0)))
#     # print del_t
#     # del_t = 0.2809
#     dels = np.linalg.inv(np.matrix([[C_p_delta_a, C_p_delta_r],[C_r_delta_a, C_r_delta_r]]))*\
#            np.matrix([[((-Gamma_1*p*q + Gamma_2*q*r)/(0.5*rho*(V_a**2)*S*b)) - C_p_0 - C_p_beta*beta - C_p_p*b*p/(2.0*V_a) - C_p_r*b*r/(2.0*V_a)]\
#            ,[((-Gamma_7*p*q + Gamma_1*q*r)/(0.5*rho*(V_a**2)*S*b)) - C_r_0 - C_r_beta*beta - C_r_p*b*p/(2.0*V_a) - C_r_r*b*r/(2.0*V_a)]])
#
#
#     del_a = dels[0,0]
#     del_r = dels[1,0]
#     states_dot_prop = propagate_(V_a,alpha,beta,phi,del_a,del_e,del_r,del_t,u,v,w,p,q,r,theta)
#     # print states_dot_prop, "states out from prop", V_a,alpha,beta,phi,del_a,del_e,del_r,del_t,u,v,w,p,q,r,theta, "variables in prop"
#     err = np.sum(abs(np.matrix(x_) - states_dot_prop))
#     print err
#     # print alpha, beta, phi, err, "abp,err"
#     return err

# def trim(V_a, R, gamma):
#     # global state
#     # print "in trim"
#
#     alpha = 0.1
#     beta = 0.0
#     phi = 0.0
#
#     #x_ is the optimum state according to given parameters. We want to minimize
#     # the error x_ - f and find what inputs do that.
#     hdot_ = V_a*np.sin(gamma)
#     psidot_ = V_a*np.cos(gamma)/R
#     x_ = [hdot_,0.0,0.0,0.0,0.0,0.0,psidot_,0.0,0.0,0.0]
#     # con = {'type': 'ineq',  \
#     #        'fun': x<3.14,
#     x_opt = scipy.optimize.minimize(f,[alpha,beta,phi],args=(V_a,gamma,x_,R),method='SLSQP')
#     # print "after optimize"
#     print(x_opt)
#     return x_opt

def update_h(msg):
     global inp_
     inp_[0][0] = msg.data

def update_va(msg):
     global inp_
     inp_[1][0] = msg.data

def update_course(msg):
     global inp_
     inp_[2][0] = msg.data

#def update_del_t(msg):
#     global inp_
#     inp_[3][0] = msg.data

def update_w_n(msg):
    global wind
    wind[0][0] = msg.data

def update_w_e(msg):
    global wind
    wind[1][0] = msg.data

def update_w_d(msg):
    global wind
    wind[2][0] = msg.data

def dynamics_timer_callback(self):
    global time_old,state, inp, wind, param, forces, moments
    # print "dynamics timer callback"
    # propagate dynamics
    time = rospy.Time.now()
    pid_calculate_command()
    propagate(time.to_sec()-time_old)
    s = np.copy(state)
    x1 = (s[0], s[1], s[2])
    x2 = (0.,0.,0.)
    x3 = (0.,0.,0.)
    x4 = (0.,0.,0.)
    q1 = tf.transformations.quaternion_from_euler(0.,0.,0.,'sxyz')
    q4 = tf.transformations.quaternion_from_euler(s[6],s[7],s[8],'sxyz')
    br.sendTransform(x1, q1, time, "veh","world")
    br.sendTransform(x4, q4, time, "base_link","veh")
    time_old = time.to_sec()
    # rospy.sleep(0.1)

def pid_calculate_command():
    global state, inp, inp_, param

    h_c = inp_[0][0]
    va_c = inp_[1][0]
    course_c = inp_[2][0]

    set_del_a_r(course_c)
    set_del_e_t(h_c,va_c)

def set_del_a_r(course_c):
    global state, inp, param, V_a, beta, alpha

    del_a_max = 3.14/3.0
    e_phi_max = 3.14/6.0
    zeta_phi = 0.707
    W_course = 8.0
    zeta_course = 0.707

    kp_phi = del_a_max/e_phi_max
    omega_phi = np.sqrt(kp_phi)
    kd_phi = 2*zeta_phi*omega_phi
    omega_course = omega_phi/W_course
    kp_course = 2*zeta_course*omega_course*V_a/9.81

    del_r_max = 3.14/6.0
    e_psi_max = 3.14
    kp_psi = del_r_max/e_psi_max

    s = np.copy(state)
    '''    p_n = s[0][0]
        p_e = s[1][0]
        p_d = s[2][0]
        u = s[3][0]
        v = s[4][0]
        w = s[5][0]
        phi = s[6][0]
        theta = s[7][0]
        psi = s[8][0]
        p = s[9][0]
        q = s[10][0]
        r = s[11][0]'''

    phi_c = kp_course*(course_c - s[8][0] - beta)
    inp[0][0] = np.sign(kp_phi*(phi_c - s[6][0]) - kd_phi*(s[9][0]))*min(np.sign(kp_phi*(phi_c - s[6][0]) - kd_phi*(s[9][0]))*(kp_phi*(phi_c - s[6][0])\
                - kd_phi*(s[9][0])),del_a_max)
    inp[2][0] = np.sign(-kp_psi*(beta))*min(np.sign(-kp_psi*(beta))*(-kp_psi*(beta)),del_r_max)
    print "phic", phi_c
    state = np.copy(s)


def set_del_e_t(h_c, va_c):
    global state, inp, param, V_a

    del_t_max = 1.0
    del_e_max = 3.14/3.0
    e_theta_max = 3.14/6.0
    zeta_theta = 0.707
    W_h = 8.0
    zeta_h = 0.8
    W_va = 8.0
    zeta_va = 0.707
    omega_t = 2.0
    zeta_t = 0.8
    h_hold = 20
    h_takeoff = 70
    theta_takeoff = 3.14/9.0
    kp_theta = -del_e_max/e_theta_max
    omega_theta = np.sqrt(abs(kp_theta) )
    omega_h = omega_theta/W_h
    kd_theta = 2*zeta_theta*omega_theta
    kp_h = 2*zeta_h*omega_h/V_a
    s = np.copy(state)

    # theta_c = kp_h*(h_c - s[2][0]) #+ ki_h*int_h_err
    # error_theta = s[7][0]
    omega_va = omega_theta/W_va
    kp_va = 2*zeta_va*omega_va
    ki_va =  np.sqrt(omega_va)
    h = -s[2][0]
    kp_t = 2*zeta_t*omega_t/9.81
    ki_t = np.sqrt(omega_t)
    theta_c = kp_h*(h_c - h)

    if h_c == 0.0 and va_c == 0.0:
        inp[3][0] = 0
        theta_c = 0
        s = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
    elif h < h_takeoff:
        inp[3][0] = 1
        theta_c = theta_takeoff
        print("h less than takeoff")
    elif h < h_c - h_hold:
        inp[3][0] = 1
        # theta_c = kp_t*(va_c - V_a)
        print("h in climb")
    elif h < h_c + h_hold:
        if inp[3][0] + kp_va*(va_c - V_a) > 0:
            inp[3][0] = min((inp[3][0] + kp_va*(va_c - V_a)),del_t_max)
        else:
            inp[3][0] = 0.0
        theta_c = kp_h*(h_c - h)
        print("h  in alt hold")
    elif h > h_c + h_hold:
        inp[3][0] = 0
        # theta_c = kp_t*(va_c - V_a)
        print("h in descend")
        # inp[1][0] = kp_theta
    print "hc",h_c,"va",va_c
    # print kp_theta, kd_theta
    inp[1][0] = np.sign(kp_theta*(theta_c - s[7][0]) + kd_theta*(s[10][0]))*min(np.sign(kp_theta*\
                (theta_c - s[7][0]) + kd_theta*(s[10][0]))*(kp_theta*(theta_c - s[7][0]) + kd_theta*(s[10][0])),del_e_max)
    state = np.copy(s)
    print "inp", inp


def propagate(dt):
    global state, inp, wind, param, forces, moments, gusts, num1, num2, num3, den1, den2, den3, t0, V_a, abp, beta, alpha
    s = np.copy(state)
    t0 +=dt
    # print state, "state starting dynamics"
    g = 9.81
    m= param['m']
    b = param['b']
    c = param['c']
    S = param['S']
    rho = param['rho']
    p_n = s[0][0]
    p_e = s[1][0]
    p_d = s[2][0]
    u = s[3][0]
    v = s[4][0]
    w = s[5][0]
    phi = s[6][0]
    theta = s[7][0]
    psi = s[8][0]
    p = s[9][0]
    q = s[10][0]
    r = s[11][0]
    origin, xaxis, yaxis, zaxis = (0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.)
    Rx = rotation_matrix(phi,xaxis)
    Ry = rotation_matrix(theta,yaxis)
    Rz = rotation_matrix(psi,zaxis)
    R_V_B = concatenate_matrices(Rx,Ry,Rz)
    R_V_B = R_V_B[0:3,0:3]
    wind_body = np.transpose(R_V_B)*np.matrix(np.copy(wind))
    ###
    # low alt, light turb
    alt = 50.0
    Lu = 200.0
    Lv = 200.0
    Lw = 50.0
    sigma_u = 1.06
    sigma_v = 1.06
    sigma_w = 0.7

    num1 = sigma_u * np.sqrt(2.0*V_a/Lu)
    den1 = [1.0, V_a/Lu]

    num2 = [sigma_v * np.sqrt(3.0*V_a/Lv), sigma_v * np.sqrt(3.0*V_a/Lv)*(V_a/np.sqrt(3.0*Lv))]
    den2 = [1.0, 2.0*V_a/Lv, (V_a/Lv)**2.0]

    num3 = [sigma_w * np.sqrt(3.0*V_a/Lw), sigma_w * np.sqrt(3.0*V_a/Lw)*(V_a/np.sqrt(3.0*Lw))]
    den3 = [1.0, 2.0*V_a/Lw, (V_a/Lw)**2.0]
    time_series = np.linspace(0,t0)
    #print np.shape(time_series)
    input_series = np.random.randn(np.size(time_series),1)*0.1
    #print time_series, input_series
    _,y1,_ = signal.lsim((num1,den1),input_series,time_series)
    _,y2,_ = signal.lsim((num2,den2),input_series,time_series)
    _,y3,_ = signal.lsim((num3,den3),input_series,time_series)
    gusts[0][0] = y1[-1]/10.0
    gusts[1][0] = y2[-1]/10.0
    gusts[2][0] = y3[-1]/10.0
    #print gusts
    # if p_d > -alt:
    #     gusts = [[0.0],[0.0],[0.0]]
    V_r = [[u-wind_body[0,0] + gusts[0][0]],[v-wind_body[1,0]+gusts[1][0]],[w-wind_body[2,0]+gusts[2][0]]]
    # V_r = [[u],[v],[w]]
    # V_a = 25.0
    V_a = np.sqrt((V_r[0][0])**2.0 + V_r[1][0]**2.0  + V_r[2][0]**2.0 )
    # print V_a, "printing V_a in propagation"
    alpha = np.arctan2(V_r[2][0],V_r[0][0])
    # alpha,beta,_ = abp
    beta = np.arcsin(V_r[1][0]/V_a)
    # alpha = np.arctan2(w,u)
    # beta = np.arcsin(v/V_a)

    sphi   = np.sin(phi)
    cphi   = np.cos(phi)
    stheta = np.sin(theta)
    ctheta = np.cos(theta)
    spsi   = np.sin(psi)
    cpsi   = np.cos(psi)
    ttheta = np.tan(theta)
    secth = 1.0/ctheta
    calpha = np.cos(alpha)
    salpha = np.sin(alpha)
    C_D_alpha = param['C_D_0'] + param['C_D_alpha']*alpha
    C_L_alpha = param['C_L_0'] + param['C_L_alpha']*alpha
    C_X = -C_D_alpha*calpha + C_L_alpha*salpha
    C_X_q = -param['C_D_q']*calpha + param['C_L_q']*salpha
    C_X_delta_e = -param['C_D_delta_e']*calpha + param['C_L_delta_e']*salpha
    C_Z = -C_D_alpha*salpha - C_L_alpha*calpha
    C_Z_q = -param['C_D_q']*salpha - param['C_L_q']*calpha
    C_Z_delta_e = -param['C_D_delta_e']*salpha - param['C_L_delta_e']*calpha
    forces = np.copy(np.matrix([[-m*g*stheta],[m*g*ctheta*sphi],[m*g*ctheta*cphi]]) + \
             0.5*rho*(V_a**2.0)*S*np.matrix([[C_X + C_X_q*c*q/(2.0*V_a)+ C_X_delta_e*inp[1][0]],\
             [param['C_Y_0'] + param['C_Y_beta']*beta + param['C_Y_p']*b*p/(2.0*V_a) + param['C_Y_r']*b*r/(2.0*V_a) + param['C_Y_delta_a']*inp[0][0] + param['C_Y_delta_r']*inp[2][0]],\
             [C_Z + C_Z_q*c*q/(2.0*V_a) + C_Z_delta_e*inp[1][0]]])\
             + 0.5*rho*param['S_prop']*param['C_prop']*np.matrix([[(param['k_motor']*inp[3][0])**2.0 - V_a**2.0],[0.],[0.]]))

    moments = np.copy(0.5*rho*(V_a**2.0)*S*np.matrix([[b*(param['C_l_0'] + param['C_l_p']*b*p/(2.0*V_a) + param['C_l_r']*b*r/(2*V_a) + param['C_l_delta_a']*inp[0][0] +\
              param['C_l_delta_r']*inp[2][0])],\
              [c*(param['C_m_0'] + param['C_m_alpha']*alpha + param['C_m_q']*c*q/(2.0*V_a) + param['C_m_delta_e']*inp[1][0])],\
              [b*(param['C_n_0'] + param['C_n_beta']*beta + param['C_n_p']*b*p/(2.0*V_a) + param['C_n_r']*b*r/(2.0*V_a) + param['C_n_delta_a']*inp[0][0] + \
              param['C_n_delta_r']*inp[2][0])]]) +\
              np.matrix([[-param['k_T_p']*((param['k_Omega']*inp[3][0])**2.0)],[0.0],[0.0]]))

    # RK4 integration
    k1 = dynamics(s)

    k2 = dynamics(s + dt/2.0*k1)
    k3 = dynamics(s + dt/2.0*k2)
    k4 = dynamics(s + dt/2.0*k3)
    temp = k1 + 2.0*k2 + 2.0*k3 + k4

    s += dt/6.0*temp

    state = np.copy(s)
    # print state, "state after dynamics and fixing trim"
    # exit()


def propagate_(V_a, alpha, beta,phi, del_a,del_e,del_r,del_t,u,v,w,p,q,r,theta):#dt,V_a, gamma,x_):
    global param, forces, moments, state #V_a # num1, num2, num3, den1, den2, den3,
    # s = np.copy(state)
    # alpha,beta,phi = A
    dt =1.0/100.0
    # dt,V_a,gamma,x_ = args
    inp[0][0] = del_a
    inp[1][0] = del_e
    inp[2][0] = del_r
    inp[3][0] = del_t

    g = 9.81
    m= param['m']
    b = param['b']
    c = param['c']
    S = param['S']
    rho = param['rho']

    sphi   = np.sin(phi)
    cphi   = np.cos(phi)
    stheta = np.sin(theta)
    ctheta = np.cos(theta)
    ttheta = np.tan(theta)
    secth = 1.0/ctheta
    calpha = np.cos(alpha)
    salpha = np.sin(alpha)
    C_D_alpha = param['C_D_0'] + param['C_D_alpha']*alpha
    C_L_alpha = param['C_L_0'] + param['C_L_alpha']*alpha
    C_X = -C_D_alpha*calpha + C_L_alpha*salpha
    C_X_q = -param['C_D_q']*calpha + param['C_L_q']*salpha
    C_X_delta_e = -param['C_D_delta_e']*calpha + param['C_L_delta_e']*salpha
    C_Z = -C_D_alpha*salpha - C_L_alpha*calpha
    C_Z_q = -param['C_D_q']*salpha - param['C_L_q']*calpha
    C_Z_delta_e = -param['C_D_delta_e']*salpha - param['C_L_delta_e']*calpha
    forces = np.copy(np.matrix([[-m*g*stheta],[m*g*ctheta*sphi],[m*g*ctheta*cphi]]) + \
             0.5*rho*(V_a**2.0)*S*np.matrix([[C_X + C_X_q*c*q/(2.0*V_a)+ C_X_delta_e*inp[1][0]],\
             [param['C_Y_0'] + param['C_Y_beta']*beta + param['C_Y_p']*b*p/(2.0*V_a) + param['C_Y_r']*b*r/(2.0*V_a) + param['C_Y_delta_a']*inp[0][0] + param['C_Y_delta_r']*inp[2][0]],\
             [C_Z + C_Z_q*c*q/(2.0*V_a) + C_Z_delta_e*inp[1][0]]])\
             + 0.5*rho*param['S_prop']*param['C_prop']*np.matrix([[(param['k_motor']*inp[3][0])**2.0 - V_a**2.0],[0.],[0.]]))

    moments = np.copy(0.5*rho*(V_a**2.0)*S*np.matrix([[b*(param['C_l_0'] + param['C_l_p']*b*p/(2.0*V_a) + param['C_l_r']*b*r/(2*V_a) + param['C_l_delta_a']*inp[0][0] +\
              param['C_l_delta_r']*inp[2][0])],\
              [c*(param['C_m_0'] + param['C_m_alpha']*alpha + param['C_m_q']*c*q/(2.0*V_a) + param['C_m_delta_e']*inp[1][0])],\
              [b*(param['C_n_0'] + param['C_n_beta']*beta + param['C_n_p']*b*p/(2.0*V_a) + param['C_n_r']*b*r/(2.0*V_a) + param['C_n_delta_a']*inp[0][0] + \
              param['C_n_delta_r']*inp[2][0])]]) +\
              np.matrix([[-param['k_T_p']*((param['k_Omega']*inp[3][0])**2.0)],[0.0],[0.0]]))
    # print forces,moments, "forces and moments"
    # RK4 integration
    s = np.copy(state)
    s[6][0] = phi
    s[3][0] = u
    s[4][0] = v
    s[5][0] = w
    s[7][0] = theta
    s[9][0] = p
    s[10][0] = q
    s[11][0] = r

    k1 = dynamics(s)
    k2 = dynamics(s + dt/2.0*k1)
    k3 = dynamics(s + dt/2.0*k2)
    k4 = dynamics(s + dt/2.0*k3)
    temp = k1 + 2.0*k2 + 2.0*k3 + k4

    return dt/6.0*temp[2:]


def dynamics(s):
    global param, inp, wind, forces, moments
    m= param['m']
    Jx = param['Jx']
    Jy = param['Jy']
    Jz = param['Jz']
    Jxz = param['Jxz']
    # Jxz = 0.0
    p_n = s[0][0]
    p_e = s[1][0]
    p_d = s[2][0]
    u = s[3][0]
    v = s[4][0]
    w = s[5][0]
    phi = s[6][0]
    theta = s[7][0]
    psi = s[8][0]
    p = s[9][0]
    q = s[10][0]
    r = s[11][0]

    state_dot = np.zeros((12,1)) #[[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]] #
    origin, xaxis, yaxis, zaxis = (0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.)
    Rx = rotation_matrix(phi,xaxis)
    Ry = rotation_matrix(theta,yaxis)
    Rz = rotation_matrix(psi,zaxis)
    R_V_B = concatenate_matrices(Rz,Ry,Rx)
    R_V_B = R_V_B[0:3,0:3]

    sphi   = np.sin(phi)
    cphi   = np.cos(phi)
    stheta = np.sin(theta)
    ctheta = np.cos(theta)
    spsi   = np.sin(psi)
    cpsi   = np.cos(psi)
    ttheta = np.tan(theta)
    secth = 1.0/ctheta
    # calpha = np.cos(alpha)
    # salpha = np.sin(alpha)

    Mat2 = np.matrix([[1.0,sphi*ttheta,cphi*ttheta],[0.,cphi,-sphi],[0.,sphi*secth,cphi*secth]])
    J = np.matrix([[Jx, 0., -Jxz],[0., Jy, 0.],[-Jxz, 0., Jz]])

    temp1 = R_V_B*np.matrix(s[3:6])
    state_dot[0:3] = temp1

    temp2 = Mat2*np.matrix(s[9:12])
    state_dot[6:9] = temp2

    temp3 = np.transpose(-np.cross(np.transpose(np.matrix(s[9:12])),np.transpose(np.matrix(s[3:6])))) + (1.0/m)*forces
    state_dot[3:6] = temp3

    temp4 = np.linalg.pinv(J)*(np.transpose(-np.cross(np.transpose(np.matrix(s[9:12])),np.transpose(J*np.matrix(s[9:12])))) + moments)
    state_dot[9:12] = temp4
    s2 = np.copy(state_dot)

    return s2


if __name__ == '__main__':
    rospy.init_node('flight_dynamics')
    br = TransformBroadcaster()
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
