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
est = [[0.0],[0.0],[0.0],[0.0],[25.0],[0.0],[0.0],[0.0],[0.0]] #phi,theta,pn,pe,Vg,chi, wn, we, psi
gusts = [[0.0],[0.0],[0.0]]
err_int_course = 0.0
prev_msgs =  [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
beta = 0.0
alpha = 0.0

def start_funct():
    global T_out, param,time_old,inp, state, V_a,P_att, P_gps,abp, pub_f, pub_pos, pub_vel, pub_att, pub_att_v, pub_acc, pub_att_a, pub_va, pub_course, course, change, vg, phi_est, theta_est
    # initialize member variables
    # print "do i get here?"
    # get parameters
    try:
        param_namespace = '/flight_dynamics'
        param = rospy.get_param(param_namespace)
    except KeyError:
        rospy.logfatal('Parameters not set in ~/flight_dynamics namespace')
        rospy.signal_shutdown('Parameters not set')
    estimation_rate = rospy.get_param('~rate', 100.0)
    T_out = 0.01
    V_a = 25.0
    course = 0.0
    change = 0
    vg = 25.0
    P_att = np.matrix([[0.5,0],[0,0.5]])
    P_gps = 0.5*np.eye(7)#np.matrix([[],[],[],[],[],[],[]])
    phi_est = 0
    theta_est = 0
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

    rospy.Subscriber("/flight_dynamics/w_n", Float32, update_w_n)
    rospy.Subscriber("/flight_dynamics/w_e", Float32, update_w_e)
    rospy.Subscriber("/flight_dynamics/w_d", Float32, update_w_d)

    time_old = rospy.Time.now().to_sec()
    rospy.Timer(rospy.Duration(1.0/estimation_rate), estimation_callback)
    rospy.spin()


def update_gps(msg):
     global state, course, vg, wind,P_att, est,V_a,P_gps
     e = np.copy(est)
     s = np.copy(state)
     w = np.copy(wind)
     g = 9.81
     m= param['m']
     b = param['b']
     c = param['c']
     S = param['S']
     rho = param['rho']
     wn = e[6][0]#w[0][0]
     we = e[5][0]#w[1][0]
     # wd = w[2][0]
     # p_n = s[0][0]
     # p_e = s[1][0]
     # p_d = s[2][0]
     # u = s[3][0]
     # v = s[4][0]
     # w = s[5][0]
     phi = e[0][0]#s[6][0]
     theta = e[1][0]#s[7][0]
     p_n = e[2][0]
     p_e = e[3][0]
     # s[2][0] = LPF(msg.linear.z)
     course = e[5][0]
     vg = e[4][0]
     #phi,theta,pn,pe,Vg,chi, wn, we, psi
     psi = e[8][0]

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
     # print vg, "vg before update", V_a, "va", course, "course", np.transpose(wind), "wind"
     Q_gps = 1e-6*np.eye(7)
     R_gps = 1e-4*np.eye(6)
     J_gps_h = np.matrix([[1.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,1.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,1.0,0.0,0.0,0.0],[0.0,0.0,-ccourse,vg*scourse,1.0,0.0,-V_a*spsi],\
                 [0.0,0.0,-scourse,-vg*ccourse,0.0,1.0,V_a*cpsi]])
     h_gps = np.matrix([[p_n],[p_e],[vg],[course],[V_a*cpsi + wn - vg*ccourse],[V_a*spsi + we - vg*scourse]])



     C_gps = J_gps_h
     L_gps = P_gps*np.transpose(C_gps)*np.linalg.inv(R_gps + C_gps*P_gps*np.transpose(C_gps))
     P_gps = (np.eye(7) - L_gps*C_gps)*P_gps
     p_n = LPF(msg.linear.x,1)
     p_e = LPF(msg.linear.y,2)
     # s[2][0] = LPF(msg.linear.z)
     course = wrap(LPF(msg.angular.y,3))
     vg = LPF(msg.angular.x,4)
     # print vg, "vg in update"
     wn = w[0][0]
     we = w[1][0]
     vals = np.matrix([[p_n],[p_e],[vg],[course],[V_a*cpsi + wn - vg*ccourse],[V_a*spsi + we - vg*scourse]])
     correction_update = L_gps*(vals - h_gps)
     if correction_update[0,0] < e[0,0] + P_att[0,0]/T_out**2 and correction_update[1,0] > e[1,0] - P_att[1,1]/T_out**2 or (P_att[0,0] < 1e-6 and P_att[1,1]<1e-6):

         correction_update[3][0] = wrap(correction_update[3][0])
         correction_update[6][0] = wrap(correction_update[6][0])

         e[2:9] = e[2:9] + np.clip(correction_update,-0.001,0.001)
     # vg = e[4][0]
     # print vg,"vg after update"

     est = np.copy(e)
     state = np.copy(s)

def wrap(a):
    if a > np.pi:
        a = a - 2*np.pi
    elif a < -np.pi:
        a = a + 2*np.pi
    else:
        a = a
    return a

def update_press(msg):
    global state, V_a, param
    s = np.copy(state)
    s[2][0] = -LPF(msg.x,5)/(param['rho']*9.81)
    V_a = np.sqrt(LPF(msg.y,6)*2.0/param['rho'])
    # print V_a, "V_a in update"
    state = np.copy(s)

def update_gyro(msg):
     global state
     s = np.copy(state)
     s[9][0] = LPF(msg.x,7)
     s[10][0] = LPF(msg.y,8)
     s[11][0] = LPF(msg.z,9)
     state = np.copy(s)

def update_accel(msg):
     global state,prev_msgs, P_att, est,T_out
     e = np.copy(est)
     s = np.copy(state)
     g = 9.81
     p = s[9][0]
     q = s[10][0]
     r = s[11][0]
     # s[6][0] = np.arctan2(msg.y,msg.z)
     # s[7][0] = np.arctan2(msg.x,g)

     theta = e[1][0]
     phi = e[2][0]
     # theta = np.arctan2(msg.y,msg.z)#s[7][0]
     # phi = np.arctan2(msg.x,g)#s[6][0]


     cphi = np.cos(phi)
     tphi = np.sin(phi)/np.cos(phi)
     sphi = np.sin(phi)
     ttheta = np.sin(theta)/np.cos(theta)
     stheta = np.sin(theta)
     ctheta = np.cos(theta)

     #parameters for attitude estimation
     # Q_att = np.matrix([[1e-5,0],[0,1e-5]])
     R_att = np.matrix([[0.0025 * 9.81 * 5/18,0,0],[0,0.0025 * 9.81* 5/18,0],[0,0,0.0025 * 9.81* 5/18]])
     h_att = np.matrix([[q*V_a*stheta + g*stheta],[r*V_a*ctheta - p*V_a*stheta - g*ctheta*sphi],[-q*V_a*ctheta - g*ctheta*cphi]])
     J_att_h = np.matrix([[0.0,q*V_a*ctheta + g*ctheta],[-g*cphi*ctheta,-r*V_a*stheta - p*V_a*ctheta + g*sphi*stheta],[g*sphi*ctheta, (q*V_a + g*cphi)*stheta]])
     C_att = J_att_h
     L_att = P_att*np.transpose(C_att)*np.linalg.inv(R_att + C_att*P_att*np.transpose(C_att))
     P_att = (np.eye(2) - L_att*C_att)*P_att
     # print C_att, L_
     vals = np.matrix([[LPF(msg.x,10)],[LPF(msg.y,11)],[LPF(msg.z,12)]])
     resid = vals - h_att
     correction_update = L_att*(resid)
     # print (P_att)
     if correction_update[0,0] < e[0,0] + P_att[0,0]/T_out**2 and correction_update[1,0] > e[1,0] - P_att[1,1]/T_out**2 or (P_att[0,0] < 1e-6 and P_att[1,1]<1e-6):
     #np.linalg.norm(resid) < 1.2*g and np.linalg.norm(resid) > 0.8*g:#correction_update[0][0] < 0.1 and correction_update[1][0] < 0.1:
         # continue
         # print "residual in theta", resid[1][0], "kalman gain for theta", [L_att[1,0], L_att[1,1], L_att[1,2]]
         correction_update[0][0] = wrap(correction_update[0][0])
         correction_update[1][0] = wrap(correction_update[1][0])
         # print "residual after wrapping", np.transpose(correction_update)
         print "updating", np.random.randn()
         # e[0:2] = e[0:2] + correction_update
         e[0:2] = e[0:2] + np.clip(correction_update,-0.0002,0.0002)

     est = np.copy(e)
     # s[6:8] = s[6:8] + L_att*(vals - h_att)
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
    return x
    '''global prev_msgs, change
    alpha_lpf = 0.2
    p = np.copy(prev_msgs)
    prev = p[mode-1][0]
    new = alpha_lpf*prev + (1-alpha_lpf)*x
    p[mode-1][0] = new
    prev_msgs = np.copy(p)
    change = 1
    return new'''

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
    global time_old,state, inp, wind, param, forces, moments, change, P_att, P_gps, Q_att, Q_gps, R_att, R_gps, pub_f, pub_pos, \
            pub_vel, pub_att, pub_att_v, pub_acc, pub_att_a, pub_va, pub_course, est
    e = np.copy(est) #phi,theta,pn,pe,Vg,chi, wn, we, psi
    time = rospy.Time.now()
    s = np.copy(state)
    w = np.copy(wind)
    g = 9.81
    m= param['m']
    b = param['b']
    c = param['c']
    S = param['S']
    rho = param['rho']

    wn = e[6][0]
    we = e[7][0]
    # wd = w[2][0]
    p_n = e[2][0]#s[0][0]
    p_e = e[3][0]#s[1][0]
    # p_d = e[4][0]#s[2][0]

    vg = e[4][0]
    chi = e[5][0]

    #get these from the estimate. These states are propagated, so the estimated state is only what we care about in propagation. #
    phi = e[0][0]#s[6][0]
    theta = e[1][0]#s[7][0]

    psi = e[8][0]

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
    Q_att = np.matrix([[1e-7,0],[0,1e-7]])
    # R_att = np.matrix([[0.0025 * 9.81 * 5/18,0,0],[0,0.0025 * 9.81* 5/18,0],[0,0,0.0025 * 9.81* 5/18]])
    f_att = np.matrix([[p + q*sphi*ttheta + r*cphi*ttheta],[q*cphi - r*sphi]])
    J_att_f = np.matrix([[q*cphi*ttheta - r*sphi*ttheta, (q*sphi + r*cphi)/(ctheta**2)],[-q*sphi - r*cphi, 0.0]])
    #predict attitude
    e[0:2] = e[0:2] + (T_out)*f_att
    A_att = J_att_f
    P_att = P_att + (T_out)*(A_att*P_att + P_att*np.transpose(A_att) + Q_att)
    # print vg,"vg before predict"
    Q_gps = 1e-6*np.eye(7)
    R_gps = 1e-3*np.eye(6)
    psi_dot = q*sphi/ctheta + r*cphi/ctheta
    vg_dot = ((V_a*cpsi + wn)*(-V_a*psi_dot*spsi) + (V_a*spsi + we)*(V_a*psi_dot*cpsi))/vg
    d_vg_dot_d_psi = (-psi_dot*V_a*(wn*cpsi + we*spsi))/vg
    d_course_dot_d_vg = -g*tphi*np.cos(course-psi)/(vg*vg)
    d_course_dot_d_course = -g*tphi*np.sin(course-psi)/vg
    d_course_dot_d_psi = g*tphi*np.sin(course-psi)/vg
    J_gps_f = np.matrix([[0.0,0.0,ccourse, -vg*scourse,0.0,0.0,0.0],[0.0,0.0,scourse,vg*ccourse,0.0,0.0,0.0],[0.0,0.0,-vg_dot/vg,0.0,-psi_dot*V_a*spsi/vg, psi_dot*V_a*cpsi/vg, d_vg_dot_d_psi],\
                [0.0,0.0,d_course_dot_d_vg, d_course_dot_d_course, 0.0,0.0, d_course_dot_d_psi],[0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0]])
    f_gps = np.matrix([[vg*ccourse],[vg*scourse],[vg_dot],[g*tphi*np.cos(course-psi)/vg],[0.0],[0.0],[psi_dot]])
    # print f_gps, e[8][0], "before updating psi"
    e[2:9] = e[2:9] + (T_out)*f_gps
    A_gps = J_gps_f
    P_gps = P_gps + (T_out)*(A_gps*P_gps + P_gps*np.transpose(A_gps) + Q_gps)
    vg = e[4][0]
    # print vg,"vg after predict"


    pos = Vector3()
    vel = Vector3()
    # acc = Vector3()
    att = Vector3()
    att_v = Vector3()
    # att_a = Vector3()

    e[0][0] = wrap(e[0][0])
    e[1][0] = wrap(e[1][0])
    e[5][0] = wrap(e[5][0])
    e[8][0] = wrap(e[8][0])

    pos.x = e[2][0]
    pos.y = e[3][0]
    pos.z = s[2][0]

    u = s[3][0]
    v = s[4][0]
    w = s[5][0]
    vel.x = u
    vel.y = v
    vel.z = w

    # print "psi at time of publishing", e[8][0]
    phi = e[0][0]
    theta = e[1][0]
    att.x = phi
    att.y = theta
    att.z = e[8][0]

    att_v.x = p
    att_v.y = q
    att_v.z = r

    # print "published everything"
    pub_pos.publish(pos)
    pub_vel.publish(vel)
    # pub_v
    # pub_acc.publish(acc)
    pub_att.publish(att)
    pub_att_v.publish(att_v)
    # pub_att_a.publish(att_a)
    pub_va.publish(vg)#V_a
    pub_course.publish(course)
    est = np.copy(e)



if __name__ == '__main__':
    rospy.init_node('ekf_pilot')
    # br = TransformBroadcaster()
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
