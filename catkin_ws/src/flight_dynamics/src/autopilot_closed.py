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
# V_a = 0.1
inp_ = [[0.00],[0.00],[0.00]]
wind = [[0.00],[0.00],[0.00]]
inp = [[0.0],[0.0],[0.0],[0.0]]#,[0.],[0.]]
# state = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
state = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
state_est =  [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
gusts = [[0.0],[0.0],[0.0]]
course_est = 0.0
err_int_course = 0.0
phic = 0.0
beta = 0.0
alpha = 0.0
closed_loop = 0.0

def start_funct():
    global param,time_old,inp, state, V_a, abp, pub_f, pub_pos, pub_vel, pub_att, pub_att_v, pub_acc, pub_att_a, pub_va, pub_course, course
    # initialize member variables
    # print "do i get here?"
    # get parameters
    try:
        param_namespace = '/flight_dynamics'
        param = rospy.get_param(param_namespace)
    except KeyError:
        rospy.logfatal('Parameters not set in ~/flight_dynamics namespace')
        rospy.signal_shutdown('Parameters not set')

    V_a = 25.0
    course = 0.0

    # rospy.Subscriber("/ekf_pilot/states_clean_forces",Vector3,)
    rospy.Subscriber("/ekf_pilot/states_clean_pos",Vector3,update_pos)
    # rospy.Subscriber("/ekf_pilot/states_clean_vel",Vector3,update)
    rospy.Subscriber("/ekf_pilot/states_clean_att",Vector3,update_att)
    rospy.Subscriber("/ekf_pilot/states_clean_att_v",Vector3,update_att_v)
    # rospy.Subscriber("/ekf_pilot/states_clean_acc",Vector3,)
    # rospy.Subscriber("/ekf_pilot/states_clean_att_a",Vector3,)
    # rospy.Subscriber("/ekf_pilot/states_clean_va",Float32,update)
    rospy.Subscriber("/ekf_pilot/states_clean_course",Float32,update_course_est)
    rospy.Subscriber("/flight_dynamics/phi", Float32, update_phi)


    pub_f = rospy.Publisher("/autopilot/states_clean_forces",Vector3,queue_size=10)
    pub_pos = rospy.Publisher("/autopilot/states_clean_pos",Vector3,queue_size=10)
    pub_vel = rospy.Publisher("/autopilot/states_clean_vel",Vector3,queue_size=10)
    pub_att = rospy.Publisher("/autopilot/states_clean_att",Vector3,queue_size=10)
    pub_att_v = rospy.Publisher("/autopilot/states_clean_att_v",Vector3,queue_size=10)
    pub_acc = rospy.Publisher("/autopilot/states_clean_acc",Vector3,queue_size=10)
    pub_att_a = rospy.Publisher("/autopilot/states_clean_att_a",Vector3,queue_size=10)
    pub_va = rospy.Publisher("/autopilot/states_clean_va",Float32,queue_size=10)
    pub_course = rospy.Publisher("/autopilot/states_clean_course",Float32,queue_size=10)
    # pub_course = rospy.Publisher("/autopilot/states_clean_course",Float32,queue_size=10)
    # pub_wind = rospy.Publisher("/autopilot/states_clean_wind",Vector3,queue_size=10)
    rospy.Subscriber("/flight_dynamics/h", Float32, update_h)
    rospy.Subscriber("/flight_dynamics/va", Float32, update_va)
    rospy.Subscriber("/flight_dynamics/course", Float32, update_course)



    rospy.Subscriber("/flight_dynamics/w_n", Float32, update_w_n)
    rospy.Subscriber("/flight_dynamics/w_e", Float32, update_w_e)
    rospy.Subscriber("/flight_dynamics/w_d", Float32, update_w_d)

    time_old = rospy.Time.now().to_sec()

    dynamics_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/dynamics_rate), dynamics_timer_callback)
    rospy.spin()

def wrap(a):
    while a > np.pi:
        a = a - 2*np.pi
    while a < -np.pi:
        a = a + 2*np.pi
    return a

def update_pos(msg):
     global state_est
     s = np.copy(state_est)
     s[0][0] = msg.x
     s[1][0] = msg.y
     s[2][0] = msg.z
     state_est = np.copy(s)

def update_att(msg):
     global state_est
     s = np.copy(state_est)
     s[6][0] = msg.x
     s[7][0] = msg.y
     s[8][0] = msg.z
     state_est = np.copy(s)

def update_att_v(msg):
     global state_est
     s = np.copy(state_est)
     # s[9][0] = msg.x
     # s[10][0] = msg.y
     # s[11][0] = msg.z
     state_est = np.copy(s)

def update_course_est(msg):
     global course_est
     course_est = msg.data



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
    # print "before propagate"
    propagate(time.to_sec()-time_old)
    # print "after propagate"
    s = np.copy(state)
    x1 = (s[0], s[1], s[2])
    x2 = (0.,0.,0.)
    x3 = (0.,0.,0.)
    x4 = (0.,0.,0.)
    # print "sending transform"
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
    global state, inp, param, V_a, beta, alpha, course, wind, err_int_course, state_est, course_est, closed_loop, phic

    del_a_max = 3.14/9.0/2.0
    e_phi_max = 3.14/6.0
    zeta_phi = 0.85
    W_course = 7.0
    zeta_course = 0.85
    i = np.copy(inp)
    kp_phi = del_a_max/e_phi_max
    omega_phi = np.sqrt(kp_phi)
    kd_phi = 2*zeta_phi*omega_phi
    omega_course = omega_phi/W_course
    kp_course = 2*zeta_course*omega_course*V_a/9.81
    phi_max = 3.14/6.0/2.0
    del_r_max = 3.14/6.0
    e_psi_max = 3.14
    kp_psi = del_r_max/e_psi_max
    if closed_loop == 1:
        s = np.copy(state_est)#np.copy(state)
    else:
        s = np.copy(state)
    psi = s[8][0]
    r = s[11][0]
    Vn = V_a*np.cos(psi) + wind[0][0]
    Ve = V_a*np.sin(psi) + wind[1][0]
    course = np.arctan2(Ve,Vn)

    ki_course = 0.1
    if r < 0.1:
        err_int_course = err_int_course + 0.01/2.0*(course_c - course)
    phi_c = kp_course*(course_c - course) + ki_course*(err_int_course) + phic
    phi_c_temp = np.sign(kp_course*(course_c - course) + ki_course*(err_int_course)+ phic)*min(np.sign(kp_course*(course_c - course) + ki_course*(err_int_course)+ phic)*\
                (kp_course*(course_c - course) + ki_course*(err_int_course)+ phic),phi_max)
    err_int_course = err_int_course + 0.01/ki_course*(phi_c_temp - phi_c)
    phi_c = np.sign(kp_course*(course_c - course) + ki_course*(err_int_course) + phic)*min(np.sign(kp_course*(course_c - course) + ki_course*(err_int_course)+ phic)*\
                (kp_course*(course_c - course) + ki_course*(err_int_course)+ phic),phi_max)



    # phi_c = np.sign(kp_course*(course_c - course) + phic)*min(np.sign(kp_course*(course_c - course) + phic)*\
    #             (kp_course*(course_c - course) + phic),phi_max)

    i[0][0] = np.sign(kp_phi*(phi_c - s[6][0]) - kd_phi*(s[9][0]))*min(np.sign(kp_phi*(phi_c - s[6][0]) - kd_phi*(s[9][0]))*(kp_phi*(phi_c - s[6][0])\
                - kd_phi*(s[9][0])),del_a_max)
    i[2][0] = np.sign(-kp_psi*(beta))*min(np.sign(-kp_psi*(beta))*(-kp_psi*(beta)),del_r_max)
    inp = np.copy(i)
    # print "phic", phi_c
    # print(i[0][0])
    # state = np.copy(s)
# def sat(a):
#     if a>lim

def set_del_e_t(h_c, va_c):
    global state, inp, param, V_a, state_est, course_est, closed_loop
    i = np.copy(inp)
    del_t_max = 0.4/2.0
    del_e_max = 3.14/6.0/2.0
    e_theta_max = 3.14/6.0
    zeta_theta = 0.707
    W_h = 8.0
    zeta_h = 0.8
    W_va = 20.0
    zeta_va = 0.8
    omega_t = 2.0
    zeta_t = 0.8
    h_hold = 50
    h_takeoff = 70
    theta_takeoff = 3.14/9.0
    kp_theta = -del_e_max/e_theta_max
    omega_theta = np.sqrt(abs(kp_theta))
    omega_h = omega_theta/W_h
    kd_theta = 2*zeta_theta*omega_theta
    kp_h = 2*zeta_h*omega_h/V_a
    if closed_loop == 1:
        s = np.copy(state_est)#np.copy(state)
    else:
        s = np.copy(state)
    # theta_c = kp_h*(h_c - s[2][0]) #+ ki_h*int_h_err
    # error_theta = s[7][0]
    omega_va = omega_theta/W_va
    kp_va = 2*zeta_va*omega_va
    ki_va =  np.sqrt(omega_va)
    h = -s[2][0]
    kp_t = 2*zeta_t*omega_t/9.81
    ki_t = np.sqrt(omega_t)
    theta_max = 3.14/4.0
    theta_c = np.sign(kp_h*(h_c - h))*min(np.sign(kp_h*(h_c - h))*kp_h*(h_c - h), theta_max)

    if h_c == 0.0 and va_c == 0.0:
        i[3][0] = 0
        theta_c = 0
        s = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
    elif h < h_takeoff:
        i[3][0] = 0.4
        theta_c = theta_takeoff
        # print("h less than takeoff")
    elif h < h_c - h_hold:
        i[3][0] = 0.4
        # theta_c = kp_t*(va_c - V_a)
        # print("h in climb")
    elif h < h_c + h_hold:
        if i[3][0] + kp_va*(va_c - V_a) > 0:
            i[3][0] = min((i[3][0] + kp_va*(va_c - V_a)),del_t_max)
        else:
            i[3][0] = 0.0
            # theta_c = kp_h*(h_c - h)
        # print("h  in alt hold")
    elif h > h_c + h_hold:
        i[3][0] = 0.0
        # theta_c = kp_t*(va_c - V_a)
        # print("h in descend")
        # inp[1][0] = kp_theta
    # print "hc",h_c,"va",va_c
    # print kp_theta, kd_theta
    i[1][0] = np.sign(kp_theta*(theta_c - s[7][0]) + kd_theta*(s[10][0]))*min(np.sign(kp_theta*\
                (theta_c - s[7][0]) + kd_theta*(s[10][0]))*(kp_theta*(theta_c - s[7][0]) + kd_theta*(s[10][0])),del_e_max)
    state = np.copy(s)
    inp = np.copy(i)
    # print "inp", inp


def propagate(dt):
    global state, inp, wind, param, forces, moments, gusts, num1, num2, num3, den1, den2, den3, t0, V_a, abp, beta, alpha, course, pub_f, pub_pos, pub_vel, pub_att, pub_att_v, pub_acc, pub_att_a, pub_va, pub_course
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
    gusts[0][0] = y1[-1]/10000.0
    gusts[1][0] = y2[-1]/10000.0
    gusts[2][0] = y3[-1]/10000.0
    #print gusts
    # if p_d > -alt:
    #     gusts = [[0.0],[0.0],[0.0]]
    V_r = [[u-wind_body[0,0] - gusts[0][0]],[v-wind_body[1,0]-gusts[1][0]],[w-wind_body[2,0]-gusts[2][0]]]
    # V_r = [[u],[v],[w]]
    # V_a = 25.0
    V_a = np.sqrt((V_r[0][0])**2.0 + V_r[1][0]**2.0  + V_r[2][0]**2.0 ) # airspeed in body frame
    # print V_a, "printing V_a in propagation"
    alpha = np.arctan2(V_r[2][0],V_r[0][0])
    # alpha,beta,_ = abp
    beta = np.arcsin(V_r[1][0]/V_a)
    # alpha = np.arctan2(w,u)
    # beta = np.arcsin(v/V_a)
    # course = psi + beta
    # course = np.atan2(v,u)
    # Vn = V_a*np.cos(psi) + wind[0][0]
    # Ve = V_a*np.sin(psi) + wind[1][0]
    #
    # course = np.arctan2(Vn,Ve)

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
    # print "calculated forces, sending left."
    forces_ = Vector3()
    forces_.x = forces[0][0]
    forces_.y = forces[1][0]
    forces_.z = forces[2][0]
    pub_f.publish(forces_)
    moments = np.copy(0.5*rho*(V_a**2.0)*S*np.matrix([[b*(param['C_l_0'] + param['C_l_p']*b*p/(2.0*V_a) + param['C_l_r']*b*r/(2*V_a) + param['C_l_delta_a']*inp[0][0] +\
              param['C_l_delta_r']*inp[2][0])],\
              [c*(param['C_m_0'] + param['C_m_alpha']*alpha + param['C_m_q']*c*q/(2.0*V_a) + param['C_m_delta_e']*inp[1][0])],\
              [b*(param['C_n_0'] + param['C_n_beta']*beta + param['C_n_p']*b*p/(2.0*V_a) + param['C_n_r']*b*r/(2.0*V_a) + param['C_n_delta_a']*inp[0][0] + \
              param['C_n_delta_r']*inp[2][0])]]) +\
              np.matrix([[-param['k_T_p']*((param['k_Omega']*inp[3][0])**2.0)],[0.0],[0.0]]))

    # print "forces and moments passed"
    # RK4 integration
    k1 = dynamics(s)

    k2 = dynamics(s + dt/2.0*k1)
    k3 = dynamics(s + dt/2.0*k2)
    k4 = dynamics(s + dt/2.0*k3)
    temp = k1 + 2.0*k2 + 2.0*k3 + k4
    temp = dt/6.0*temp

    # s2 = np.copy(state_dot)
    pos = Vector3()
    vel = Vector3()
    acc = Vector3()
    att = Vector3()
    att_v = Vector3()
    att_a = Vector3()
    # print "initialised all vector 3s"
    pos.x = p_n
    pos.y = p_e
    pos.z = p_d

    vel.x = u#temp[0][0]
    vel.y = v#temp[1][0]
    vel.z = w#temp[2][0]

    acc.x = temp[3][0]
    acc.y = temp[4][0]
    acc.z = temp[5][0]

    att.x = phi
    att.y = theta
    att.z = psi

    att_v.x = p#temp[6][0]
    att_v.y = q#temp[7][0]
    att_v.z = r#temp[8][0]

    att_a.x = temp[9][0]
    att_a.y = temp[10][0]
    att_a.z = temp[11][0]

    # print "published everything"
    pub_pos.publish(pos)
    pub_vel.publish(vel)
    pub_acc.publish(acc)
    pub_att.publish(att)
    pub_att_v.publish(att_v)
    pub_att_a.publish(att_a)
    pub_va.publish(V_a)
    pub_course.publish(course)
    # print "published eberything "
    # pub_wind.publish(wind)
    s += temp

    s[6][0] = wrap(s[6][0])
    s[7][0] = wrap(s[7][0])
    s[8][0] = wrap(s[8][0])
    state = np.copy(s)
    # exit()
    # print state, "state after dynamics and fixing trim"
    # exit()


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
