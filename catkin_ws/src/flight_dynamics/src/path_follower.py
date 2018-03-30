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



wind = [[0.00],[0.00],[0.00]]
state = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
mode = -1
V_a = 35.0

def start_funct():
    global state,pub_h, pub_va, pub_chi, pub_phi

    try:
        param_namespace = '/flight_dynamics'
        param = rospy.get_param(param_namespace)
    except KeyError:
        rospy.logfatal('Parameters not set in ~/flight_dynamics namespace')
        rospy.signal_shutdown('Parameters not set')

    pub_h = rospy.Publisher("/flight_dynamics/h",Float32,queue_size=10)
    pub_va = rospy.Publisher("/flight_dynamics/va",Float32,queue_size=10)
    pub_chi = rospy.Publisher("/flight_dynamics/course",Float32,queue_size=10)
    pub_phi = rospy.Publisher("/flight_dynamics/phi",Float32,queue_size=10)

    rospy.Subscriber("/autopilot/states_clean_pos",Vector3,update_pos)
    rospy.Subscriber("/autopilot/states_clean_att",Vector3,update_att)
    rospy.Subscriber("/new_autopilot/va_", Float32, update_va)

    rospy.Subscriber("/flight_dynamics/mode", Float32, update_mode)

    rospy.Subscriber("/flight_dynamics/w_n", Float32, update_w_n)
    rospy.Subscriber("/flight_dynamics/w_e", Float32, update_w_e)
    rospy.Subscriber("/flight_dynamics/w_d", Float32, update_w_d)

    dynamics_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/dynamics_rate), run_path_follower)
    rospy.spin()


def run_path_follower(self):
    global mode
    if mode>=0:
        follow_straight_line(what_to_do(1))
    elif mode<0:
        follow_orbit(what_to_do(2))


def what_to_do(mode):
    if mode==1:
        Va_d   = 35.
        r      = np.matrix([[0.],[0.],[-100.]])
        q      = np.matrix([[-0.5],[-1.],[-0.05]])
        q      = q/np.linalg.norm(q)
        return (Va_d, r, q, None)
    elif mode==2:
        Va_d   = 35.
        c      = np.matrix([[1.],[1.],[-100.]])
        rho    = 400.
        l = 1.
        return(Va_d, c, rho, l)

def update_va(msg):
    global V_a
    V_a = msg.data

def update_pos(msg):
     global state
     s = np.copy(state)
     s[0][0] = msg.x
     s[1][0] = msg.y
     s[2][0] = msg.z
     state = np.copy(s)


def update_att(msg):
     global state
     s = np.copy(state)
     s[6][0] = msg.x
     s[7][0] = msg.y
     s[8][0] = msg.z
     state = np.copy(s)


def update_mode(msg):
    global mode
    mode = msg.data

def follow_straight_line(tup):
    global wind, state, pub_h, pub_va, pub_chi, V_a, pub_phi
    print "line follower"
    kpath = 0.01
    chi_inf = np.pi/6.

    Vad = tup[0]

    r = tup[1]
    rn = r[0][0]
    re = r[1][0]
    rd = r[2][0]

    q = tup[2]
    qn = q[0][0]
    qe = q[1][0]
    qd = q[2][0]

    s = np.copy(state)
    we = wind[1][0]
    wn = wind[0][0]
    chi = s[8][0]
    pn = s[0][0]
    pe = s[1][0]
    pd = s[2][0]

    #Vn = Vad*np.cos(psi) + wn
    #Ve = V_a*np.sin(psi) + we
    #chi = np.arctan2(Ve,Vn)

    chi_q = np.arctan2(q[1],q[0])
    # chi_q = wrap(chi_q-chi)
    while chi_q - chi > np.pi:
        chi_q = chi_q - 2*np.pi
    while chi_q - chi < -np.pi:
        chi_q = chi_q + 2*np.pi
    # return a

    # print s[0:3].T , r
    R = np.matrix([[np.cos(chi_q),np.sin(chi_q),0.],[-np.sin(chi_q),np.cos(chi_q),0.],[0.,0.,1.]])
    err = s[0:3] - r
    ep = R*err
    k = np.matrix([[0.],[0.],[1.]])
    n = np.cross(q.T,k.T)/np.linalg.norm(np.cross(q.T,k.T))
    n = n.T
    s_ = err - ((n.T*err)*n.T).T

    sn = s_[0][0]
    se = s_[1][0]
    sd = s_[2][0]

    e_py = -np.sin(chi_q)*(pn - rn) + np.cos(chi_q)*(pe - re)
    h_c = -rd + np.sqrt(sn**2 + se**2)*qd/(np.sqrt(qn**2 + qe**2))
    chi_c = wrap(chi_q - chi_inf*2*(np.arctan2(kpath*e_py,1))/np.pi)
    va_c = Vad
    phi_ff = 0.0
    print chi_c, chi
    # print phi_ff, h_c, va_c, chi_c
    print e_py
    pub_phi.publish(phi_ff)
    pub_h.publish(h_c)
    pub_va.publish(va_c)
    pub_chi.publish(chi_c)


def follow_orbit(tup):
    global wind, state, h_c, chi_c, pub_h, pub_va, pub_chi, V_a, pub_phi
    print "orbit follower"

    korbit = 0.9
    g = 9.81
    Vad = tup[0]
    c = tup[1]
    cn = c[0][0]
    ce = c[1][0]
    cd = c[2][0]

    rho = tup[2]
    l = tup[3]

    s = np.copy(state)
    we = wind[1][0]
    wn = wind[0][0]
    chi = s[8][0]
    pn = s[0][0]
    pe = s[1][0]
    pd = s[2][0]

    #Vn = V_a*np.cos(psi) + wn
    #Ve = V_a*np.sin(psi) + we
    #print Vn, Ve, "Vn,Ve"
    #chi = np.arctan2(Ve,Vn)

    # print pn, pe, pd
    d = np.sqrt((pn - cn)**2 + (pe - ce)**2)
    psi = np.arctan2(pe - ce, pn - cn)
    # psi = wrap(psi-chi)

    while psi - chi > np.pi:
        psi = psi - 2*np.pi
    while psi - chi < -np.pi:
        psi = psi + 2*np.pi
    # return a
    # tphi = np.sin(phic)
    phi_ff = l*np.arctan2(Vad**2/(g*rho),1.)
    h_c = -cd
    chi_c = psi + l*(np.pi/2. + np.arctan2(korbit*(d-rho)/(rho),1.))
    va_c = Vad

    # print phi_ff, h_c, va_c, chi_c
    print d
    print chi_c, chi
    pub_phi.publish(phi_ff)
    pub_h.publish(h_c)
    pub_va.publish(va_c)
    pub_chi.publish(chi_c)


def wrap(a):
    while a > np.pi:
        a = a - 2*np.pi
    while a < -np.pi:
        a = a + 2*np.pi
    return a

def update_w_n(msg):
    global wind
    wind[0][0] = msg.data

def update_w_e(msg):
    global wind
    wind[1][0] = msg.data

def update_w_d(msg):
    global wind
    wind[2][0] = msg.data



if __name__ == '__main__':
    rospy.init_node('path_follower')
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
