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
gusts = [[0.0],[0.0],[0.0]]
err_int_course = 0.0
prev_msgs =  [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
beta = 0.0
alpha = 0.0
T_out_att = 1/80.0

def start_funct():
    global param,time_old,inp, state, V_a,P_att, P_gps,abp, pub_f, pub_pos, pub_vel, pub_att, pub_att_v, pub_acc, pub_att_a, pub_va, pub_course, course, change, vg
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
    change = 0
    vg = 25.0
    P_att = np.matrix([[0.5,0],[0,0.5]])
    P_gps = 0.5*np.eye(7)#np.matrix([[],[],[],[],[],[],[]])

    pub_f = rospy.Publisher("/ekf_pilot/states_clean_forces",Vector3,queue_size=10)
    pub_pos = rospy.Publisher("/ekf_pilot/states_clean_pos",Vector3,queue_size=10)
    pub_vel = rospy.Publisher("/ekf_pilot/states_clean_vel",Vector3,queue_size=10)
    pub_att = rospy.Publisher("/ekf_pilot/states_clean_att",Vector3,queue_size=10)
    pub_att_v = rospy.Publisher("/ekf_pilot/states_clean_att_v",Vector3,queue_size=10)
    pub_acc = rospy.Publisher("/ekf_pilot/states_clean_acc",Vector3,queue_size=10)
    pub_att_a = rospy.Publisher("/ekf_pilot/states_clean_att_a",Vector3,queue_size=10)
    pub_va = rospy.Publisher("/ekf_pilot/states_clean_va",Float32,queue_size=10)
    pub_course = rospy.Publisher("/ekf_pilot/states_clean_course",Float32,queue_size=10)

    rospy.Subscriber("/sensors/gps",Twist,update_gps)
    rospy.Subscriber("/sensors/gyro",Vector3,update_gyro)
    rospy.Subscriber("/sensors/pressure",Vector3,update_press)
    rospy.Subscriber("/sensors/accel",Vector3,update_accel)


    # pub_course = rospy.Publisher("/autopilot/states_clean_course",Float32,queue_size=10)
    # pub_wind = rospy.Publisher("/autopilot/states_clean_wind",Vector3,queue_size=10)
    # rospy.Subscriber("/flight_dynamics/h", Float32, update_h)
    # rospy.Subscriber("/flight_dynamics/va", Float32, update_va)
    # rospy.Subscriber("/flight_dynamics/course", Float32, update_course)



    rospy.Subscriber("/flight_dynamics/w_n", Float32, update_w_n)
    rospy.Subscriber("/flight_dynamics/w_e", Float32, update_w_e)
    rospy.Subscriber("/flight_dynamics/w_d", Float32, update_w_d)

    time_old = rospy.Time.now().to_sec()

    estimation_rate = rospy.get_param('~rate', 80.0)
    rospy.Timer(rospy.Duration(1.0/estimation_rate), estimation_callback)
    rospy.spin()


def update_gps(msg):
     global state, course, vg,change
     s = np.copy(state)
     s[0][0] = LPF(msg.linear.x,1)
     s[1][0] = LPF(msg.linear.y,2)
     # s[2][0] = LPF(msg.linear.z)
     course = LPF(msg.angular.x,3)
     vg = LPF(msg.angular.y,4)
     change = 1
     state = np.copy(s)

def update_press(msg):
    global state, V_a, param
    s = np.copy(state)
    s[2][0] = LPF(msg.x,5)/(param['rho']*9.81)
    V_a = np.sqrt(LPF(msg.y,6)*2.0/param['rho'])
    state = np.copy(s)

def update_gyro(msg):
     global state
     s = np.copy(state)
     s[9][0] = LPF(msg.x,7)
     s[10][0] = LPF(msg.y,8)
     s[11][0] = LPF(msg.z,9)
     state = np.copy(s)

def update_accel(msg):
     global state,prev_msgs,change
     p = np.copy(prev_msgs)
     s = np.copy(state)
     '''
     theta = s[7][0]
     phi = s[6][0]'''
     # s[6][0] = np.arctan2(LPF(msg.y,10),LPF(msg.z,11))
     # s[7][0] = np.arctan2(LPF(msg.x,12),9.81)
     s[6][0] = np.arctan2(LPF(msg.y,11),LPF(msg.z,12))
     s[7][0] = np.arctan2(LPF(msg.x,10),9.81)
     change = 1
     # p[9][0] = msg.x
     # p[10][0] = msg.y
     # p[11][0] = msg.z
     prev_msgs = np.copy(p)
     # s[0][0] = LPF(msg.linear.x)
     # s[0][1] = LPF(msg.linear.y)
     # s[2][0] = LPF(msg.linear.z)
     state = np.copy(s)

'''
1: gps_n
2: gps_e
3: gps_course
4: gps_vg
5: press_alt
6: press_va
7: gyro_x
8: gyro_y
9: gyro_z
10: acc_x
11: acc_y
12: acc_z
'''


def LPF(x,mode):
    global prev_msgs, change
    alpha_lpf = 0.2
    p = np.copy(prev_msgs)
    prev = p[mode-1][0]
    new = alpha_lpf*prev + (1-alpha_lpf)*x
    p[mode-1][0] = new
    prev_msgs = np.copy(p)
    return new

# def update_h(msg):
#      global inp_
#      inp_[0][0] = msg.data
#
# def update_va(msg):
#      global inp_
#      inp_[1][0] = msg.data
#
# def update_course(msg):
#      global inp_
#      inp_[2][0] = msg.data

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

def estimation_callback(self):
    global time_old,state, inp, wind, param, forces, moments, change, P_att, P_gps, Q_att, Q_gps, R_att, R_gps, pub_f, pub_pos, pub_vel, pub_att, pub_att_v, pub_acc, pub_att_a, pub_va, pub_course
    time = rospy.Time.now()
    s = np.copy(state)
    w = np.copy(wind)
    g = 9.81
    m= param['m']
    b = param['b']
    c = param['c']
    S = param['S']
    rho = param['rho']
    wn = w[0][0]
    we = w[1][0]
    wd = w[2][0]
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

    cphi = np.cos(phi)
    tphi = np.sin(phi)/np.cos(phi)
    sphi = np.sin(phi)
    ttheta = np.sin(theta)/np.cos(theta)
    stheta = np.sin(theta)
    ctheta = np.cos(theta)
    ccourse = np.cos(course)
    scourse = np.sin(course)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    #parameters for attitude estimation
    Q_att = np.matrix([[1e-8,0],[0,1e-8]])
    R_att = np.matrix([[0.0025 * 9.81 * 5/18,0,0],[0,0.0025 * 9.81* 5/18,0],[0,0,0.0025 * 9.81* 5/18]])

    f_att = np.matrix([[p + q*sphi*ttheta + r*cphi*ttheta],[q*cphi - r*sphi]])
    J_att_f = np.matrix([[q*cphi*ttheta - r*sphi*ttheta, (q*sphi + r*cphi)/(ctheta*ctheta)],[-q*sphi - r*cphi, 0.0]])

    h_att = np.matrix([[q*V_a*stheta + g*stheta],[r*V_a*ctheta - p*V_a*stheta - g*ctheta*sphi],[-q*V_a*ctheta - g*ctheta*cphi]])
    J_att_h = np.matrix([[0.0,q*V_a*ctheta + g*ctheta],[-g*cphi*ctheta,-r*V_a*stheta - p*V_a*ctheta + g*sphi*stheta],[g*sphi*ctheta, (q*V_a + g*cphi)*stheta]])

    # predict_att()

    Q_gps = 1e-10*np.eye(7)
    R_gps = 1e-2*np.eye(6)
    psi_dot = q*sphi/ctheta + r*cphi/ctheta
    vg_dot = ((V_a*cpsi + wn)*(-V_a*psi_dot*spsi) + (V_a*spsi + we)*(V_a*psi_dot*cpsi))/vg
    d_vg_dot_d_psi = (-psi_dot*V_a*(wn*cpsi + we*spsi))/vg
    d_course_dot_d_vg = -g*tphi*np.cos(course-psi)/(vg*vg)
    d_course_dot_d_course = -g*tphi*np.sin(course-psi)/vg
    d_course_dot_d_psi = g*tphi*np.sin(course-psi)/vg
    J_gps_f = np.matrix([[0.0,0.0,ccourse, -vg*scourse,0.0,0.0,0.0],[0.0,0.0,scourse,vg*ccourse,0.0,0.0,0.0],[0.0,0.0,-vg_dot/vg,0.0,-psi_dot*V_a*spsi/vg, psi_dot*V_a*cpsi/vg, d_vg_dot_d_psi],\
                [0.0,0.0,d_course_dot_d_vg, d_course_dot_d_course, 0.0,0.0, d_course_dot_d_psi],[0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0]])
    J_gps_h = np.matrix([[1.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,1.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,1.0,0.0,0.0,0.0],[0.0,0.0,-ccourse,vg*scourse,1.0,0.0,-V_a*spsi],\
                [0.0,0.0,-scourse,-vg*ccourse,0.0,1.0,V_a*cpsi]])
    f_gps = np.matrix([[vg*ccourse],[vg*scourse],[vg_dot],[g*tphi*np.cos(course-psi)/vg],[0.0],[0.0],[psi_dot]])
    h_gps = np.matrix([[p_n],[p_e],[vg],[course],[V_a*cpsi + wn - vg*ccourse],[V_a*spsi + we - vg*scourse]])

    # s = np.copy(state)
    predict(J_att_f,f_att,J_gps_f,f_gps,Q_att,Q_gps,P_att,P_gps)
    # print change
    if change==1:
        print "going in update"
        update(J_att_h, h_att, J_gps_h, h_gps, R_att, R_gps, P_att, P_gps)
        # change = 0
    # state = s.copy()
    publish_all()

def publish_all():
    global state, course, V_a, vg, pub_f, pub_pos, pub_vel, pub_att, pub_att_v, pub_acc, pub_att_a, pub_va, pub_course, change
    # if change == 0:
    s = np.copy(state)
# if change == 0:
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

    pos = Vector3()
    vel = Vector3()
    # acc = Vector3()
    att = Vector3()
    att_v = Vector3()
    # att_a = Vector3()
    # print "initialised all vector 3s"
    pos.x = p_n
    pos.y = p_e
    pos.z = p_d
    vel.x = u
    vel.y = v
    vel.z = w
    # acc.x = temp[3][0]
    # acc.y = temp[4][0]
    # acc.z = temp[5][0]
    att.x = phi
    att.y = theta
    att.z = psi
    att_v.x = p
    att_v.y = q
    att_v.z = r
    # att_a.x = temp[9][0]
    # att_a.y = temp[10][0]
    # att_a.z = temp[11][0]
    # print "published everything"
    pub_pos.publish(pos)
    pub_vel.publish(vel)
    # pub_acc.publish(acc)
    pub_att.publish(att)
    pub_att_v.publish(att_v)
    # pub_att_a.publish(att_a)
    pub_va.publish(V_a)
    pub_course.publish(course)
    # time_old = time.to_sec()
    # rospy.sleep(0.1)


def predict(J_att_f,f_att,J_gps_f,f_gps,Q_att,Q_gps,P_att,P_gps):
    global state, V_a, vg, course, param, wind,T_out_att
    s = np.copy(state)
    N_att = 1
    # print np.transpose(f_att), s[6:8,0]
    s[6:8] = s[6:8] + (T_out_att/N_att)*f_att
    A_att = J_att_f
    P_att = P_att + (T_out_att/N_att)*(A_att*P_att + P_att*np.transpose(A_att) + Q_att)
    # print "in prediction"

    #TODO: gps equations


    state = np.copy(s)


'''
1: gps_n
2: gps_e
3: gps_course
4: gps_vg
5: press_alt
6: press_va
7: gyro_x
8: gyro_y
9: gyro_z
10: acc_x
11: acc_y
12: acc_z
'''

def update(J_att_h, h_att, J_gps_h, h_gps, R_att, R_gps, P_att, P_gps):
    global state, V_a, vg, course, param, wind,T_out, prev_msgs
    s = np.copy(state)
    p = np.copy(prev_msgs)
    # N_att = 2
    C_att = J_att_h
    L_att = P_att*np.transpose(C_att)*np.linalg.inv(R_att + C_att*P_att*np.transpose(C_att))
    P_att = (np.eye(2) - L_att*C_att)*P_att
    # print L_att
    # if np.sum(abs(L_att*(p[9:12] - h_att)))<0.1:
    # s[6:8] = s[6:8] + L_att*(p[9:12] - h_att)
    s[6:8] = s[6:8] + np.clip(L_att*(p[9:12] - h_att),-0.025,0.025)
    # print "in update, residual: ",np.transpose(np.clip(L_att*(p[9:12] - h_att),-0.02,0.02)), np.transpose(s[6:8])
    state = np.copy(s)
    change = 0
    # else:
    #     print "measurement rejected"

if __name__ == '__main__':
    rospy.init_node('ekf_pilot')
    # br = TransformBroadcaster()
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
