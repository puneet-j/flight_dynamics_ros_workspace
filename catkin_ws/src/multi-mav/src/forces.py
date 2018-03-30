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




#sys1 = signal.TransferFunction(num1,den1)
t0 = 0.0
V_a = 0.1

wind = [[0.01],[0.00],[0.00]]
inp = [[0.0],[0.],[0.0],[0.0]]#,[0.],[0.]]
# state = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
state = [[0.0],[0.0],[-200.0],[30.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
gusts = [[0.0],[0.0],[0.0]]

def start_funct():
    global param,time_old,inp, state
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

    rospy.Subscriber("/flight_dynamics/del_a", Float32, update_del_a)
    rospy.Subscriber("/flight_dynamics/del_e", Float32, update_del_e)
    rospy.Subscriber("/flight_dynamics/del_r", Float32, update_del_r)
    rospy.Subscriber("/flight_dynamics/del_t", Float32, update_del_t)

    rospy.Subscriber("/flight_dynamics/w_n", Float32, update_w_n)
    rospy.Subscriber("/flight_dynamics/w_e", Float32, update_w_e)
    rospy.Subscriber("/flight_dynamics/w_d", Float32, update_w_d)

    time_old = rospy.Time.now().to_sec()
    # print "hi i am after subscribers"
    # setup simulation timer
    dynamics_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/dynamics_rate), dynamics_timer_callback)
    rospy.spin()

def update_del_a(msg):
    global inp
    inp[0][0] = msg.data

def update_del_e(msg):
    global inp
    inp[1][0] = msg.data

def update_del_r(msg):
    global inp
    inp[2][0] = msg.data

def update_del_t(msg):
    global inp
    inp[3][0] = msg.data

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
    # print time_old, time.to_sec()
    # time.sleep(10)
    # rospy.sleep(10)
    propagate(time.to_sec()-time_old)
    s = np.copy(state)
    # pub = rospy.Publisher('pos', Twist, queue_size=10)
    # pub2 = rospy.Publisher('vel', Twist, queue_size=10)
    # # rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # pos = Twist()
    # pos.linear.x =
    # pub.publish(pos)
    # pub2.publish(vel)
    # # print state
    x1 = (s[0], s[1], s[2])
    x2 = (0.,0.,0.)
    x3 = (0.,0.,0.)
    x4 = (0.,0.,0.)
    q1 = tf.transformations.quaternion_from_euler(0.,0.,0.,'sxyz')
    # q2 = tf.transformations.quaternion_from_euler(0.,0.,s[8],'sxyz')
    # q3 = tf.transformations.quaternion_from_euler(0.,s[7],0.,'sxyz')
    # q4 = tf.transformations.quaternion_from_euler(s[6],0.,0.,'sxyz')
    q4 = tf.transformations.quaternion_from_euler(s[6],s[7],s[8],'sxyz')
    br.sendTransform(x1, q1, time, "veh","world")
    # br.sendTransform(x2, q2, time, "v1","veh")
    # br.sendTransform(x3,q3,time,"v2","v1")
    br.sendTransform(x4, q4, time, "base_link","veh")


    # br.sendTransform(x1, q4, time, "base_link","world")
    time_old = time.to_sec()
    # rospy.sleep(0.1)

def propagate(dt):
    global state, inp, wind, param, forces, moments, gusts, num1, num2, num3, den1, den2, den3, t0, V_a
    s = np.copy(state)
    t0 +=dt

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
    # Hu = sigma_u * np.sqrt(2*Va/Lu) * 1/(s+(Va/Lu))
    # Hv = sigma_v * np.sqrt(3*Va/Lv) * (s + (Va/np.sqrt(3*Lv)))/(s+(Va/Lv))**2
    # Hw = sigma_w * np.sqrt(3*Va/Lw) * (s + (Va/np.sqrt(3*Lw)))/(s+(Va/Lw))**2

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
    gusts[0][0] = y1[-1]
    gusts[1][0] = y2[-1]
    gusts[2][0] = y3[-1]
    #print gusts
    if p_d > -alt:
        gusts = [[0.0],[0.0],[0.0]]
    V_r = [[u-wind_body[0,0] + gusts[0][0]],[v-wind_body[1,0]+gusts[1][0]],[w-wind_body[2,0]+gusts[2][0]]]
    V_r = [[u],[v],[w]]
    #gusts =
    V_a = np.sqrt((V_r[0][0])**2.0 + V_r[1][0]**2.0  + V_r[2][0]**2.0 )
    alpha = np.arctan2(V_r[2][0],V_r[0][0])
    beta = np.arcsin(V_r[1][0]/V_a)


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
    #print moments, b,V_a,p,r,inp[0][0],inp[2][0]

    # RK4 integration
    k1 = dynamics(s)
    # print k1
    # print dt/2.0
    # print dt/2.0*k1
    # print "dynamics step 2", s + dt/2.0*k1
    k2 = dynamics(s + dt/2.0*k1)
    # print "dynamics step 3", s + dt/2.0*k2
    k3 = dynamics(s + dt/2.0*k2)
    # print "dynamics step 4", s + dt/2.0*k3
    k4 = dynamics(s + dt/2.0*k3)
    temp = k1 + 2.0*k2 + 2.0*k3 + k4
    # print temp
    s += dt/6.0*temp

    state = np.copy(s)


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
    # wind_body = np.transpose(R_V_B)*np.matrix(np.copy(wind))
    # V_r = [[u-wind_body[0,0]],[v-wind_body[1,0]],[w-wind_body[2,0]]]
    # print u
    # print wind_body
    # print V_r[0][0]
    # print inp[0][0]
    # V_a = np.sqrt((V_r[0][0])**2.0 + V_r[1][0]**2.0  + V_r[2][0]**2.0 )
    # alpha = np.arctan2(V_r[2][0],V_r[0][0])
    # beta = np.arcsin(V_r[1][0]/V_a)
    # print Jx
    # print R_V_B, np.transpose(R_V_B)
    # print u,v,w,V_r
    # print V_a, alpha, beta
    # print inp
    # time.sleep(10)
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
    # C_D_alpha = param['C_D_0'] + param['C_D_alpha']*alpha
    # C_L_alpha = param['C_L_0'] + param['C_L_alpha']*alpha
    # C_X = -C_D_alpha*calpha + C_L_alpha*salpha
    # C_X_q = -param['C_D_q']*calpha + param['C_L_q']*salpha
    # C_X_delta_e = -param['C_D_delta_e']*calpha + param['C_L_delta_e']*salpha
    # C_Z = -C_D_alpha*salpha - C_L_alpha*calpha
    # C_Z_q = -param['C_D_q']*salpha - param['C_L_q']*calpha
    # C_Z_delta_e = -param['C_D_delta_e']*salpha - param['C_L_delta_e']*calpha
    # forces = np.matrix([[-m*g*stheta],[m*g*ctheta*sphi],[m*g*ctheta*cphi]]) + \
    #          0.5*rho*(V_a**2.0)*S*np.matrix([[C_X + C_X_q*c*q/(2.0*V_a)+ C_X_delta_e*inp[1][0]],\
    #          [param['C_Y_0'] + param['C_Y_beta']*beta + param['C_Y_p']*b*p/(2.0*V_a) + param['C_Y_r']*b*r/(2.0*V_a) + param['C_Y_delta_a']*inp[0][0] + param['C_Y_delta_r']*inp[2][0]],\
    #          [C_Z + C_Z_q*c*q/(2.0*V_a) + C_Z_delta_e*inp[1][0]]])\
    #          + 0.5*rho*param['S_prop']*param['C_prop']*np.matrix([[(param['k_motor']*inp[3][0])**2.0 - V_a**2.0],[0.],[0.]])
    #
    # moments = 0.5*rho*(V_a**2.0)*S*np.matrix([[b*(param['C_l_0'] + param['C_l_p']*b*p/(2.0*V_a) + param['C_l_r']*b*r/(2*V_a) + param['C_l_delta_a']*inp[0][0] + param['C_l_delta_r']*inp[2][0])],\
    #           [c*(param['C_m_0'] + param['C_m_alpha']*alpha + param['C_m_q']*c*q/(2.0*V_a) + param['C_m_delta_e']*inp[1][0])],\
    #           [b*(param['C_n_0'] + param['C_n_beta']*beta + param['C_n_p']*b*p/(2.0*V_a) + param['C_n_r']*b*r/(2.0*V_a) + param['C_n_delta_a']*inp[0][0] + param['C_n_delta_r']*inp[2][0])]]) +\
    #           np.matrix([[-param['k_T_p']*((param['k_Omega']*inp[3][0])**2.0)],[0.],[0.]])
    #

    temp1 = R_V_B*np.matrix(s[3:6])
    state_dot[0:3] = temp1

    temp2 = Mat2*np.matrix(s[9:12])
    state_dot[6:9] = temp2

    temp3 = np.transpose(-np.cross(np.transpose(np.matrix(s[9:12])),np.transpose(np.matrix(s[3:6])))) + (1.0/m)*forces
    state_dot[3:6] = temp3

    temp4 = np.linalg.pinv(J)*(np.transpose(-np.cross(np.transpose(np.matrix(s[9:12])),np.transpose(J*np.matrix(s[9:12])))) + moments)
    state_dot[9:12] = temp4
    # if p_d<0:
    #     state_dot[2] = 0
    #     state_dot[5] = 0

    s2 = np.copy(state_dot)

    # print 's',s,'f', forces,'moments',moments
    # print 'sdot', s2#, 't1', temp1, 't2', temp2, 't3', temp3, 't4', temp4
    # time.sleep(1)

    return s2


if __name__ == '__main__':
    rospy.init_node('flight_dynamics')
    br = TransformBroadcaster()
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
