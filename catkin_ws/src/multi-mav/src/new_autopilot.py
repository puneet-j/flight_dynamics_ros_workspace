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
wind = [[0.1],[0.1],[0.00]]
# inp = [[0.0],[0.0],[0.0],[0.0]]#,[0.],[0.]]
# state = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
state = [[0.0],[0.0],[0.0],[0.0],[25.0],[0.0],[0.0],[0.0],[0.0]]# [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
# state_est =  [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
# s_dot_vec =  [[0.0],[0.0],[0.0],[0.0],[0.0]]
gusts = [[0.0],[0.0],[0.0]]
# course_est = 0.0
# err_int_course = 0.0
psi = 0.1
chi_old = 0.0
hc_old = 0.0
h_old = 0.0
chi_c_old = 0.0
bva = 0.1
bh = 0.1
bchi = 0.1
bchidot = 0.1
bhdot = 0.1
# time_old =  rospy.Time.now().to_sec()
def start_funct():
    global param,time_old,state, pub_h, pub_va, pub_chi
    # initialize member variables
    # print "do i get here?"
    # get parameters
    try:
        param_namespace = '/flight_dynamics'
        param = rospy.get_param(param_namespace)
    except KeyError:
        rospy.logfatal('Parameters not set in ~/flight_dynamics namespace')
        rospy.signal_shutdown('Parameters not set')

    #V_a = 25.0
    #course = 0.0
    time_old = rospy.Time.now().to_sec()

    pub_h =  rospy.Publisher("/new_autopilot/h_", Float32, queue_size=10)
    pub_va =  rospy.Publisher("/new_autopilot/va_", Float32, queue_size=10)
    pub_chi =  rospy.Publisher("/new_autopilot/chi_", Float32, queue_size=10)


    rospy.Subscriber("/flight_dynamics/bchidot", Float32, update_bchidot)
    rospy.Subscriber("/flight_dynamics/bchi", Float32, update_bchi)
    rospy.Subscriber("/flight_dynamics/bhdot", Float32, update_bhdot)
    rospy.Subscriber("/flight_dynamics/bh", Float32, update_bh)
    rospy.Subscriber("/flight_dynamics/bva", Float32, update_bva)

    rospy.Subscriber("/flight_dynamics/h", Float32, update_h)
    rospy.Subscriber("/flight_dynamics/va", Float32, update_va)
    rospy.Subscriber("/flight_dynamics/course", Float32, update_course)

    dynamics_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/dynamics_rate), dynamics_timer_callback)
    rospy.spin()

def propagate(dt):
    global state,psi,chi_old,hc_old,h_old,inp_,chi_c_old
    s = np.copy(state)
    # t0 +=dt
    # print state, "state starting dynamics"
    g = 9.81
    # RK4 integration
    k1 = dynamics(s)
    k2 = dynamics(s + dt/2.0*k1)
    k3 = dynamics(s + dt/2.0*k2)
    k4 = dynamics(s + dt/2.0*k3)
    temp = k1 + 2.0*k2 + 2.0*k3 + k4
    temp = dt/6.0*temp
    temp[3][0] = wrap(temp[3][0])
    # s_dot_vec = np.copy(temp)
    s += temp
    s[3][0] = wrap(s[3][0])
    #exit()
    #chi_old = wrap(s[3][0])
    hc_old = inp_[0][0]
    #h_old = -s[2][0]
    chi_c_old = inp_[2][0]
    state = np.copy(s)

def dynamics(s):
    global state,psi,chi_old,hc_old,h_old,inp_,chi_c_old, bh, bva, bhdot, bchidot, bchi
    m= param['m']
    Jx = param['Jx']
    Jy = param['Jy']
    Jz = param['Jz']
    Jxz = param['Jxz']

    p_n = s[0][0]##
    p_e = s[1][0]##
    V_a = s[4][0]##
    chi_dot = wrap(s[5][0])##
    h_dot = s[6][0]##

    h = -s[2][0]
    chi = wrap(s[3][0])
    #print(h,h_dot)
    chi_c_dot = wrap(inp_[2][0] - chi_c_old)
    hc_dot = inp_[0][0] - hc_old
    #if inp_[1][0] == 0:
    #    inp_[1][0] = 0.001

    w_n = wind[0][0]
    w_e = wind[1][0]
    val = -np.sin(chi)*w_n + np.cos(chi)*w_e
    psi = wrap(chi - np.arcsin(val/V_a))
    #print (w_n, w_e,p_n, p_e,  h, V_a, psi)
    state_dot = np.zeros((9,1)) #[[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]] #
    state_dot[0][0] = V_a*np.cos(psi) + w_n
    state_dot[1][0] = V_a*np.sin(psi) + w_e
    state_dot[2][0] = h_dot
    state_dot[3][0] = chi_dot
    state_dot[4][0] = bva*(inp_[1][0]-V_a)
    state_dot[5][0] = bchidot*(chi_c_dot-chi_dot) + bchi*(inp_[2][0] - chi)
    state_dot[6][0] = -(bhdot*(hc_dot - h_dot) +  bh*(inp_[0][0] - h))
    # print (chi_c_dot-chi_dot, inp_[2][0] - chi, chi)
    # if chi_c_dot != 0.0:
    #     print(chi_c_dot, "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    # else:
    #     print("chi, chi_dot, chi_c, chi_c_dot, sat chi_dot, sat chi_dot_dot, V_a, h")
    #     print(chi,chi_dot, inp_[2][0], chi_c_dot, state_dot[3][0], state_dot[5][0], V_a, h)
    #state_dot[7][0] = chi_c_dot
    #state_dot[8][0] = hc_dot
    s2 = state_dot.copy()
    return s2

def sat(a,val):
    if a > val:
        a = val
    elif a < -val:
        a = -val
    else:
        a = val
    return a

def wrap(a):
    if a > np.pi:
        a = a - 2*np.pi
    elif a < -np.pi:
        a = a + 2*np.pi
    else:
        a = a
    return a


def update_h(msg):
     global inp_
     inp_[0][0] = msg.data
def update_va(msg):
     global inp_
     inp_[1][0] = msg.data

def update_course(msg):
     global inp_
     inp_[2][0] = msg.data



def update_bh(msg):
     global bh
     bh = msg.data

def update_bva(msg):
     global bva
     bva = msg.data

def update_bchi(msg):
     global bchi
     bchi = msg.data

def update_bchidot(msg):
     global bchidot
     bchidot = msg.data

def update_bhdot(msg):
     global bhdot
     bhdot = msg.data


def dynamics_timer_callback(self):
    global state,psi, time_old, pub_h, pub_va, pub_chi
    # print "dynamics timer callback"
    # propagate dynamics
    time = rospy.Time.now()
    # pid_calculate_command()
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
    # q4 = tf.transformations.quaternion_from_euler(s[6],s[7],s[8],'sxyz')
    #if psi == 0:
    #    psi = 0.001
    pub_h.publish(s[2][0])
    pub_va.publish(s[4][0])
    pub_chi.publish(s[3][0])
    q4 = tf.transformations.quaternion_from_euler(0.,0.,psi,'sxyz')
    # print psi, "quat", q1,q4

    # br.sendTransform(x1, q1, time, "veh","world")
    # br.sendTransform(x4, q4, time, "base_link","veh")
    time_old = time.to_sec()
    # rospy.sleep(0.1)



if __name__ == '__main__':
    rospy.init_node('new_autopilot')
    br = TransformBroadcaster()
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
