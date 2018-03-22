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



eta_gps_n = 0.21
eta_gps_e = 0.21
eta_gps_d = 0.40
eta_gps = [eta_gps_n, eta_gps_e, eta_gps_d]
k_gps = 1/1100.0
T_s = 10.0
state = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
mu = [0.0,0.0,0.0]
forces_ = [0.0,0.0,0.0]
g = 9.81
'''            p_n = s[0][0]
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

            '''

accels = [0.0,0.0,0.0]
accels_att = [0.0,0.0,0.0]
wind = [[0.0],[0.0],[0.0]]

def start_funct():
    global param,time_old,inp, state, V_a, abp, pub_gps, pub_gyro, pub_accel, pub_press, course
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
    rospy.Subscriber("/autopilot/states_clean_pos",Vector3,update_pos)
    rospy.Subscriber("/autopilot/states_clean_vel",Vector3,update_vel)
    rospy.Subscriber("/autopilot/states_clean_att",Vector3,update_att)
    rospy.Subscriber("/autopilot/states_clean_att_v",Vector3,update_att_v)
    rospy.Subscriber("/autopilot/states_clean_acc",Vector3,update_acc)
    rospy.Subscriber("/autopilot/states_clean_att_a",Vector3,update_att_a)
    rospy.Subscriber("/autopilot/states_clean_va",Float32,update_va)
    rospy.Subscriber("/autopilot/states_clean_course",Float32,update_course)
    #rospy.Subscriber("/autopilot/states_clean_wind",Vector3,update_wind)
    rospy.Subscriber("/autopilot/states_clean_forces",Vector3,update_forces)

    pub_gps = rospy.Publisher("/sensors/gps",Twist,queue_size=10)
    pub_gyro = rospy.Publisher("/sensors/gyro",Vector3,queue_size=10)
    pub_press = rospy.Publisher("/sensors/pressure",Vector3,queue_size=10)
    pub_accel = rospy.Publisher("/sensors/accel",Vector3,queue_size=10)
    #pub_acc = rospy.Publisher("/sensors/states_clean_acc",Vector3,queue_size=10)
    #pub_att_a = rospy.Publisher("/sensors/states_clean_att_a",Vector3,queue_size=10)

    #
    rospy.Subscriber("/flight_dynamics/w_n", Float32, update_w_n)
    rospy.Subscriber("/flight_dynamics/w_e", Float32, update_w_e)
    rospy.Subscriber("/flight_dynamics/w_d", Float32, update_w_d)

    time_old = rospy.Time.now().to_sec()

    gps_rate = rospy.get_param('~rate', 10.0)
    rate_gyros_rate = rospy.get_param('~rate',80.0)
    accel_rate = rospy.get_param('~rate',80.0)
    pressure_rate = rospy.get_param('~rate',100.0)
    rospy.Timer(rospy.Duration(1.0/gps_rate), gps)
    rospy.Timer(rospy.Duration(1.0/rate_gyros_rate), rate_gyros)
    rospy.Timer(rospy.Duration(1.0/accel_rate), accel)
    rospy.Timer(rospy.Duration(1.0/pressure_rate), pressure)

    rospy.spin()

'''
eta_gps_n = 0.21
eta_gps_e = 0.21
eta_gps_d = 0.40
eta_gps = [eta_gps_n, eta_gps_e, eta_gps_d]
k_gps = 1/1100.0
T_s = 1.0
state = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]
mu = [0.0,0.0,0.0]
'''

def gps(self):
    global eta_gps, k_gps, T_s, state, mu, course, vg, V_a, pub_gps
    s = np.copy(state)
    mu = np.exp(-k_gps*T_s)*np.matrix(mu) + np.matrix([eta_gps])
    # print s[0:3,0], mu
    s[0:3,0] = s[0:3,0] + mu
    psi = s[8,0]
    # print psi
    vn = V_a*np.cos(psi) + wind[0][0]
    ve = V_a*np.sin(psi) + wind[1][0]
    vg = np.sqrt(vn**2 + ve**2)
    # course = np.atan2(ve,vn)
    sigma_vg = 0.05
    sigma_course = 0.05/vg

    vg = vg + sigma_vg*np.random.randn()
    course = course + sigma_course*np.random.randn()
    # sigma_vg = np.sqrt((vn**2 * )/())
    pos = Twist()
    pos.linear.x = s[0,0]
    pos.linear.y = s[1,0]
    pos.linear.z = - s[2,0]
    pos.angular.x = vg
    pos.angular.y = course
    pos.angular.z = 0.0
    pub_gps.publish(pos)
    # state = np.copy(s)

def rate_gyros(self):
    global state, pub_gyro
    s = np.copy(state)
    # sigma_g = np.sqrt(90)*0.015
    sigma_g = 0.13*3.14/180
    p = s[9][0]
    q = s[10][0]
    r = s[11][0]
    gyros = Vector3()
    gyros.x = p + sigma_g*np.random.randn()
    gyros.y = q + sigma_g*np.random.randn()
    gyros.z = r + sigma_g*np.random.randn()
    pub_gyro.publish(gyros)

def accel(self):
    global forces_, state, g, param, pub_accel
    s = np.copy(state)
    theta = s[7][0]
    phi = s[6][0]
    m = param['m']
    # sigma_a = np.sqrt(100)*250*0.000001*0.001
    sigma_a = 0.0025 * 9.81
    acc = Vector3()
    acc.x = (forces_[0]/m) + g*np.sin(theta) + sigma_a*np.random.randn()
    acc.y = (forces_[1]/m) - g*np.cos(theta)*np.sin(phi) + sigma_a*np.random.randn()
    acc.z = (forces_[2]/m) - g*np.cos(theta)*np.cos(phi) + sigma_a*np.random.randn()
    pub_accel.publish(acc)

def pressure(self):
    global V_a, param, state, pub_press
    s = np.copy(state)
    beta_absp = 0.125
    sigma_absp = 0.01
    beta_diffp = 0.020
    sigma_diffp = 0.002
    rho = param['rho']
    h = -s[2][0]
    alt_press = rho*g*h + beta_absp + sigma_absp*np.random.randn()
    air_press = rho*(V_a**2)*0.5 + beta_diffp + sigma_diffp*np.random.randn()

    press = Vector3()
    press.x = alt_press
    press.y = air_press
    press.z = 0.0
    pub_press.publish(press)

'''    pub_gps = rospy.Publisher("/sensors/gps",Vector3,queue_size=10)
    pub_gyro = rospy.Publisher("/sensors/gyro",Vector3,queue_size=10)
    pub_press = rospy.Publisher("/sensors/pressure",Vector3,queue_size=10)
    pub_accel = rospy.Publisher("/sensors/accel",Vector3,queue_size=10)'''




def update_pos(msg):
    global state
    s = np.copy(state)
    s[0,0] = msg.x
    s[1,0] = msg.y
    s[2,0] = msg.z
    state = np.copy(s)

def update_vel(msg):
    global state
    s = np.copy(state)
    s[3,0] = msg.x
    s[4,0] = msg.y
    s[5,0] = msg.z
    state = np.copy(s)

def update_acc(msg):
    global accels
    # s = np.copy(state)
    accels[0] = msg.x
    accels[1] = msg.y
    accels[2] = msg.z
    # state = np.copy(s)

def update_att(msg):
    global state
    s = np.copy(state)
    s[6,0] = msg.x
    s[7,0] = msg.y
    s[8,0] = msg.z
    state = np.copy(s)

def update_att_v(msg):
    global state
    s = np.copy(state)
    s[9,0] = msg.x
    s[10,0] = msg.y
    s[11,0] = msg.z
    state = np.copy(s)

def update_att_a(msg):
    global accels_att
    # accels_att = msg.data
    # global state
    # s = np.copy(state)
    # s[0,9:12] = msg.data
    # state = np.copy(s)

def update_va(msg):
    global V_a
    # s = np.copy(state)
    V_a = msg.data
    # state = np.copy(s)

def update_course(msg):
    global course
    # s = np.copy(state)
    course = msg.data
    # state = np.copy(s)

def update_forces(msg):
    global forces_
    forces_[0] = msg.x
    forces_[1] = msg.y
    forces_[2] = msg.z

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
    rospy.init_node('sensors')
    #
    try:
        start_funct()
    except:
        rospy.ROSInterruptException
    pass
