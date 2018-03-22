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
state = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[25.0]]#,[0.0],[0.0]]# [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
# state_est =  [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
# s_dot_vec =  [[0.0],[0.0],[0.0],[0.0],[0.0]]
gusts = [[0.0],[0.0],[0.0]]
'''
p_n = s[0][0]##
p_e = s[1][0]##
psi = s[2][0]
phi = s[3][0]
h = -s[4][0]
h_dot = s[5][0]
V_a = s[6][0]
'''
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
bphi = 0.1
# time_old =  rospy.Time.now().to_sec()
def start_funct():
    global param,time_old, state, bphi
    # initialize member variables
    # print "do i get here?"
    # get parameters
    try:
        param_namespace = '/flight_dynamics'
        param = rospy.get_param(param_namespace)
    except KeyError:
        rospy.logfatal('Parameters not set in ~/flight_dynamics namespace')
        rospy.signal_shutdown('Parameters not set')

    time_old = rospy.Time.now().to_sec()


    # rospy.Subscriber("/flight_dynamics/bchidot", Float32, update_bchidot)
    rospy.Subscriber("/flight_dynamics/bphi", Float32, update_bphi)
    rospy.Subscriber("/flight_dynamics/bhdot", Float32, update_bhdot)
    rospy.Subscriber("/flight_dynamics/bh", Float32, update_bh)
    rospy.Subscriber("/flight_dynamics/bva", Float32, update_bva)

    rospy.Subscriber("/flight_dynamics/h", Float32, update_h)
    rospy.Subscriber("/flight_dynamics/va", Float32, update_va)
    rospy.Subscriber("/flight_dynamics/phi", Float32, update_phi)

    dynamics_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/dynamics_rate), dynamics_timer_callback)
    rospy.spin()

def propagate(dt):
    global state,psi,chi_old,hc_old,h_old,inp_,chi_c_old
    s = np.copy(state)
    g = 9.81
    # RK4 integration
    k1 = dynamics(s)
    k2 = dynamics(s + dt/2.0*k1)
    k3 = dynamics(s + dt/2.0*k2)
    k4 = dynamics(s + dt/2.0*k3)
    temp = k1 + 2.0*k2 + 2.0*k3 + k4
    temp = dt/6.0*temp
    temp[2][0] = wrap(temp[2][0])
    temp[3][0] = wrap(temp[3][0])
    # s_dot_vec = np.copy(temp)
    s += temp
    s[3][0] = wrap(s[3][0])
    s[2][0] = wrap(s[2][0])

    hc_old = inp_[0][0]
    state = np.copy(s)

def dynamics(s):
    global state,psi,chi_old,hc_old,h_old,inp_,chi_c_old, bh, bva, bhdot, bphi
    m= param['m']
    Jx = param['Jx']
    Jy = param['Jy']
    Jz = param['Jz']
    Jxz = param['Jxz']
    g = 9.81
    p_n = s[0][0]##
    p_e = s[1][0]##
    psi = s[2][0]#
    phi = s[3][0]#
    h = -s[4][0]
    h_dot = s[5][0]#
    V_a = s[6][0]#

    hc_dot = inp_[0][0] - hc_old

    tphi = np.sin(phi)/np.cos(phi)
    w_n = wind[0][0]
    w_e = wind[1][0]

    state_dot = np.zeros((7,1))
    state_dot[0][0] = V_a*np.cos(psi) + w_n
    state_dot[1][0] = V_a*np.sin(psi) + w_e
    state_dot[2][0] = g*tphi/V_a
    state_dot[3][0] = bphi*(inp_[2][0] - phi)
    state_dot[4][0] = h_dot
    state_dot[5][0] = -(bhdot*(hc_dot - h_dot) +  bh*(inp_[0][0] - h))
    state_dot[6][0] = bva*(inp_[1][0] - V_a)
    print(np.transpose(inp_), "input")
    print (inp_[2][0] - phi, phi)
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

def update_phi(msg):
     global inp_
     inp_[2][0] = msg.data



def update_bh(msg):
     global bh
     bh = msg.data

def update_bva(msg):
     global bva
     bva = msg.data

def update_bphi(msg):
     global bphi
     bphi = msg.data

# def update_bchidot(msg):
#      global bchidot
#      bchidot = msg.data

def update_bhdot(msg):
     global bhdot
     bhdot = msg.data


def dynamics_timer_callback(self):
    global state,psi, time_old
    # print "dynamics timer callback"
    # propagate dynamics
    time = rospy.Time.now()
    # pid_calculate_command()
    # print "before propagate"
    propagate(time.to_sec()-time_old)
    # print "after propagate"
    s = np.copy(state)
    x1 = (s[0], s[1], s[4])
    x2 = (0.,0.,0.)
    x3 = (0.,0.,0.)
    x4 = (0.,0.,0.)

    psi = s[2][0]
    phi = s[3][0]

    # print "sending transform"
    q1 = tf.transformations.quaternion_from_euler(0.,0.,0.,'sxyz')
    # q4 = tf.transformations.quaternion_from_euler(s[6],s[7],s[8],'sxyz')
    #if psi == 0:
    #    psi = 0.001

    q4 = tf.transformations.quaternion_from_euler(phi,0.,psi,'sxyz')
    # print psi, "quat", q1,q4

    br.sendTransform(x1, q1, time, "veh","world")
    br.sendTransform(x4, q4, time, "base_link","veh")
    time_old = time.to_sec()
    # rospy.sleep(0.1)



if __name__ == '__main__':
    rospy.init_node('autopilot')
    br = TransformBroadcaster()
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
