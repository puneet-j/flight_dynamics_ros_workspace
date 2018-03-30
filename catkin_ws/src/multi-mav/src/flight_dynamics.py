#!/usr/bin/python


import rospy
import numpy as np
from std_msgs.msg import Float32
from tf.broadcaster import TransformBroadcaster
import tf
from tf.transformations import rotation_matrix, concatenate_matrices, euler_from_matrix

inp = [[0.],[0.],[0.],[0.],[0.],[0.]]
# state = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
state = [[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]]

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
    rospy.Subscriber("/flight_dynamics/posx", Float32, update_posx)
    # print "hi"
    rospy.Subscriber("/flight_dynamics/posy", Float32, update_posy)
    rospy.Subscriber("/flight_dynamics/posz", Float32, update_posz)
    rospy.Subscriber("/flight_dynamics/torx", Float32, update_torx)
    rospy.Subscriber("/flight_dynamics/tory", Float32, update_tory)
    rospy.Subscriber("/flight_dynamics/torz", Float32, update_torz)
    time_old = 0.0
    # print "hi i am after subscribers"
    # setup simulation timer
    dynamics_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/dynamics_rate), dynamics_timer_callback)
    rospy.spin()

def update_posx(msg):
    global inp
    inp[0][0] = msg.data

def update_posy(msg):
    global inp
    inp[1][0] = msg.data

def update_posz(msg):
    global inp
    inp[2][0] = msg.data

def update_torx(msg):
    global inp
    inp[3][0] = msg.data

def update_tory(msg):
    global inp
    inp[4][0] = msg.data

def update_torz(msg):
    global inp
    inp[5][0] = msg.data


def dynamics_timer_callback(self):
    global time_old,state
    # print "dynamics timer callback"
    # propagate dynamics
    time = rospy.Time.now()
    propagate(time.to_sec()-time_old)
    s = np.copy(state)
    # print state
    # x1 = (s[0], s[1], s[2])
    # # x2 = (0.,0.,0.)
    # # x3 = (0.,0.,0.)
    # # q1 = tf.transformations.quaternion_from_euler(0.,0.,0.,'sxyz')
    # # q2 = tf.transformations.quaternion_from_euler(0.,0.,state[8],'sxyz')
    # # q3 = tf.transformations.quaternion_from_euler(state[6],state[7],0.,'sxyz')
    # # q4 = tf.transformations.quaternion_from_euler(s[6],s[7],s[8],'sxyz')
    # # br.sendTransform(x1, q1, time, "v1","world")
    # # br.sendTransform(x2, q2, time, "v2","v1")
    # # br.sendTransform(x3, q3, time, "base_link","v2")
    # br.sendTransform(x1, q4, time, "base_link","world")


    x1 = (s[0], s[1], s[2])
    x2 = (0.,0.,0.)
    x3 = (0.,0.,0.)
    x4 = (0.,0.,0.)
    # q1 = tf.transformations.quaternion_from_euler(0.,0.,0.,'sxyz')
    # q2 = tf.transformations.quaternion_from_euler(0.,0.,s[8],'sxyz')
    # q3 = tf.transformations.quaternion_from_euler(0.,s[7],0.,'sxyz')
    # q4 = tf.transformations.quaternion_from_euler(s[6],0.,0.,'sxyz')
    q4 = tf.transformations.quaternion_from_euler(s[6],s[7],s[8],'sxyz')
    # br.sendTransform(x1, q1, time, "veh","world")
    # br.sendTransform(x2, q2, time, "v1","veh")
    # br.sendTransform(x3,q3,time,"v2","v1")
    # br.sendTransform(x4, q4, time, "base_link","v2")
    br.sendTransform(x1, q4, time, "base_link","world")
    time_old = time.to_sec()
    # rospy.sleep(0.1)

def propagate(dt):
    global state
    s = np.copy(state)
    # RK4 integration
    k1 = dynamics(s)
    k2 = dynamics(s + dt/2.0*k1)
    k3 = dynamics(s + dt/2.0*k2)
    k4 = dynamics(s + dt/2.0*k3)
    temp = k1 + 2.0*k2 + 2.0*k3 + k4
    s += dt/6.0*temp

    state = np.copy(s)


def dynamics(s):
    global param, inp
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
    sphi   = np.sin(phi)
    cphi   = np.cos(phi)
    stheta = np.sin(theta)
    ctheta = np.cos(theta)
    spsi   = np.sin(psi)
    cpsi   = np.cos(psi)
    ttheta = np.tan(theta)
    secth = 1.0/ctheta
    # fx = inp[0]
    # fy = inp[1]
    # fz = inp[2]
    # print phi
    J = np.matrix([[Jx, 0., -Jxz],[0., Jy, 0.],[-Jxz, 0., Jz]])

    Rx = rotation_matrix(phi,xaxis)
    Ry = rotation_matrix(theta,yaxis)
    Rz = rotation_matrix(psi,zaxis)
    R_V_B = concatenate_matrices(Rz,Ry,Rx)
    R_V_B = R_V_B[0:3,0:3]

    Mat2 = np.matrix([[1.0,sphi*ttheta,cphi*ttheta],[0.,cphi,-sphi],[0.,sphi*secth,cphi*secth]])
    # print R_V_B
    # print state[3:6]
    temp = R_V_B*np.matrix(s[3:6])
    # print "temp1",temp
    state_dot[0:3] = temp #[[temp[0]],[temp[1]],[temp[2]]]
    temp = Mat2*np.matrix(s[9:12])
    # print "temp2",temp
    state_dot[6:9] = temp#[[temp[0]],[temp[1]],[temp[2]]]
    temp = np.transpose(-np.cross(np.transpose(np.matrix(s[9:12])),np.transpose(np.matrix(s[3:6])))) + (1.0/m)*np.matrix(inp[0:3])
    # print "temp3",temp
    state_dot[3:6] = temp#[[temp[0]],[temp[1]],[temp[2]]]
    temp = np.linalg.pinv(J)*(np.transpose((-np.cross(np.transpose(np.matrix(s[9:12])),np.transpose(J*np.matrix(s[9:12]))))) + np.matrix(inp[3:6]))
    # print "temp4",temp
    state_dot[9:12] = temp#[[temp[0]],[temp[1]],[temp[2]]]
    # print "inp", inp, "state", state, "state_dot", state_dot
    s2 = np.copy(state_dot)

    print "s",s2
    return s2


if __name__ == '__main__':
    rospy.init_node('flight_dynamics')
    br = TransformBroadcaster()
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
