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
from geometry_msgs.msg import Vector3




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
phic = 0.0
err_int_course = 0.0
del_a_max = 3.14/3.0


def start_funct():
    global param,time_old,inp, state, V_a, pub_h, pub_va, pub_chi, pub_phi, pub_pos, pub_att
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
    V_a = 25.0
    # print "in start"
    pub_h = rospy.Publisher("/autopilot/h",Float32,queue_size=10)
    pub_va = rospy.Publisher("/autopilot/va",Float32,queue_size=10)
    pub_phi = rospy.Publisher("/autopilot/phi",Float32,queue_size=10)
    pub_chi = rospy.Publisher("/autopilot/chi",Float32,queue_size=10)

    pub_pos = rospy.Publisher("/autopilot/states_clean_pos",Vector3,queue_size=10)
    pub_att = rospy.Publisher("/autopilot/states_clean_att",Vector3,queue_size=10)

    rospy.Subscriber("/flight_dynamics/h", Float32, update_h)
    rospy.Subscriber("/flight_dynamics/va", Float32, update_va)
    rospy.Subscriber("/flight_dynamics/course", Float32, update_course)
    rospy.Subscriber("/flight_dynamics/phi", Float32, update_phi)

    rospy.Subscriber("/flight_dynamics/w_n", Float32, update_w_n)
    rospy.Subscriber("/flight_dynamics/w_e", Float32, update_w_e)
    rospy.Subscriber("/flight_dynamics/w_d", Float32, update_w_d)

    time_old = rospy.Time.now().to_sec()
    # print "here"
    dynamics_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/dynamics_rate), dynamics_timer_callback)
    rospy.spin()


def update_h(msg):
     global inp_
     inp_[0][0] = msg.data

def update_va(msg):
     global inp_
     inp_[1][0] = msg.data

def update_course(msg):
     global inp_
     inp_[2][0] = msg.data


def update_phi(msg):
     global phic
     phic = msg.data


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
    # print "here"
    # print 'in pid calc command'
    pid_calculate_command()
    # print "here2"
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
    global state, inp, param, V_a, beta, alpha, wind,err_int_course, phic

    del_a_max = 3.14/3.0
    e_phi_max = 3.14/6.0
    zeta_phi = 0.707
    W_course = 5.0
    zeta_course = 0.8
    # print("start of del_a_r")
    kp_phi = del_a_max/e_phi_max
    omega_phi = np.sqrt(kp_phi)
    kd_phi = 2*zeta_phi*omega_phi
    omega_course = omega_phi/W_course
    kp_course = 2*zeta_course*omega_course*V_a/9.81
    phi_max = 3.14/6.0
    del_r_max = 3.14/6.0
    e_psi_max = 3.14
    kp_psi = del_r_max/e_psi_max
    s = np.copy(state)
    psi = s[8][0]
    r = s[11][0]
    Vn = V_a*np.cos(psi) + wind[0][0]
    Ve = V_a*np.sin(psi) + wind[1][0]
    course = np.arctan2(Ve,Vn)
    # print "setting del a r"
    # ki_course = 0.1
    # if r < 0.1:
    #     err_int_course = err_int_course + 0.01/2.0*(course_c - course)
    # #
    # phi_c = kp_course*(course_c - course) + ki_course*(err_int_course)+ phic
    # phi_c_temp = np.sign(kp_course*(course_c - course) + ki_course*(err_int_course)+ phic)*min(np.sign(kp_course*(course_c - course) + ki_course*(err_int_course)+ phic)*\
    #             (kp_course*(course_c - course) + ki_course*(err_int_course)+ phic),phi_max)
    # err_int_course = err_int_course + 0.01/ki_course*(phi_c_temp - phi_c)
    #
    # phi_c = kp_course*(course_c - course) + ki_course*(err_int_course)
    # phic = phi_c

    phi_c = np.sign(kp_course*(course_c - course) + phic)*min(np.sign(kp_course*(course_c - course)+ phic)*\
                (kp_course*(course_c - course) + phic),phi_max)
    # phi_c = np.sign(kp_course*(course_c - course) + ki_course*(err_int_course) )*min(np.sign(kp_course*(course_c - course) + ki_course*(err_int_course))*\
    #             (kp_course*(course_c - course) + ki_course*(err_int_course)),phi_max)
    # print phi_c, beta, err_int_course
    inp[0][0] = np.sign(kp_phi*(phi_c - s[6][0]) - kd_phi*(s[9][0]))*min(np.sign(kp_phi*(phi_c - \
                s[6][0]) - kd_phi*(s[9][0]))*(kp_phi*(phi_c - s[6][0])\
                - kd_phi*(s[9][0])),del_a_max)
    print course_c, phi_c, s[6][0], kp_phi, kd_phi, inp[0][0]
    inp[2][0] = np.sign(-kp_psi*(beta))*min(np.sign(-kp_psi*(beta))*(-kp_psi*(beta)),del_r_max)
    # print("end of del_a_r")
    # print "phic", phi_c
    # state = np.copy(s)


def set_del_e_t(h_c, va_c):
    global state, inp, param, V_a
    # print("start of del_e_t")

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
    h_hold = 50
    h_takeoff = 70
    theta_takeoff = 3.14/9.0
    kp_theta = -del_e_max/e_theta_max
    omega_theta = np.sqrt(abs(kp_theta) )
    omega_h = omega_theta/W_h
    kd_theta = 2*zeta_theta*omega_theta
    kp_h = 2*zeta_h*omega_h/V_a
    s = np.copy(state)

    omega_va = omega_theta/W_va
    kp_va = 2*zeta_va*omega_va
    ki_va =  np.sqrt(omega_va)
    h = -s[2][0]
    kp_t = 2*zeta_t*omega_t/9.81
    ki_t = np.sqrt(omega_t)
    theta_max = 3.14/4.0
    theta_c = np.sign(kp_h*(h_c - h))*min(np.sign(kp_h*(h_c - h))*kp_h*(h_c - h), theta_max)
    # print "set del e t"
    if h_c == 0.0 and va_c == 0.0:
        inp[3][0] = 0
        theta_c = 0
        s = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
    elif h < h_takeoff:
        inp[3][0] = 1
        theta_c = theta_takeoff
        # print("h less than takeoff")
    elif h < h_c - h_hold:
        inp[3][0] = 1
        # print("h in climb")
    elif h < h_c + h_hold:
        if inp[3][0] + kp_va*(va_c - V_a) > 0:
            inp[3][0] = min((inp[3][0] + kp_va*(va_c - V_a)),del_t_max)
        else:
            inp[3][0] = 0.0
        # print("h  in alt hold")
    elif h > h_c + h_hold:
        inp[3][0] = 0
        # print("h in descend")

    inp[1][0] = np.sign(kp_theta*(theta_c - s[7][0]) + kd_theta*(s[10][0]))*min(np.sign(kp_theta*\
                (theta_c - s[7][0]) + kd_theta*(s[10][0]))*(kp_theta*(theta_c - s[7][0]) + kd_theta*(s[10][0])),del_e_max)
    # print("end of del_e_t")



def propagate(dt):
    global state, inp, wind, param, forces, moments, gusts, num1, num2, num3, den1, den2, den3, t0, V_a, abp, beta,\
        alpha, pub_h, pub_va, pub_phi, pub_chi, pub_pos, pub_att
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
    gusts[0][0] = y1[-1]/100.0
    gusts[1][0] = y2[-1]/100.0
    gusts[2][0] = y3[-1]/100.0
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
    Vn = V_a*np.cos(psi) + wind[0][0]
    Ve = V_a*np.sin(psi) + wind[1][0]
    course = np.arctan2(Ve,Vn)
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
    # print "propagate"
    k2 = dynamics(s + dt/2.0*k1)
    k3 = dynamics(s + dt/2.0*k2)
    k4 = dynamics(s + dt/2.0*k3)
    temp = k1 + 2.0*k2 + 2.0*k3 + k4

    s += dt/6.0*temp
    state = np.copy(s)


    pos = Vector3()
    att = Vector3()

    pos.x = s[0][0]
    pos.y = s[1][0]
    pos.z = s[2][0]

    att.x = s[6][0]
    att.y = s[7][0]
    att.z = s[8][0] #phi

    pub_h.publish(s[2][0])
    pub_va.publish(V_a)
    pub_phi.publish(s[6][0])
    pub_chi.publish(course)

    pub_pos.publish(pos)
    pub_att.publish(att)




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
    rospy.init_node('autopilot')
    br = TransformBroadcaster()
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
