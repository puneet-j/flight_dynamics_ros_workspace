#!/usr/bin/python

import rospy
import copy
from std_msgs.msg import Float32
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, PoseStamped
from tf.broadcaster import TransformBroadcaster
import tf
import numpy as np
from random import random
from math import sin
# from pyquaternion import Quaternion

inp = [0.,0.,0.,0.,0.,0.]
n = 0.0
e = 0.0
d = 0.0
q = np.array([0,0,0,1.0])

def frameCallback():
    rospy.Subscriber("/flight_dynamics/posx", Float32, update_posx)
    rospy.Subscriber("/flight_dynamics/posy", Float32, update_posy)
    rospy.Subscriber("/flight_dynamics/posz", Float32, update_posz)
    rospy.Subscriber("/flight_dynamics/torx", Float32, update_torx)
    rospy.Subscriber("/flight_dynamics/tory", Float32, update_tory)
    rospy.Subscriber("/flight_dynamics/torz", Float32, update_torz)

    run_rate = rospy.get_param('~rate', 100.0)
    rospy.Timer(rospy.Duration(1.0/run_rate),send_transform)
    rospy.spin()

def send_transform(self):
    time = rospy.Time.now()
    q = tf.transformations.quaternion_from_euler(inp[3],inp[4],inp[5],'sxyz')
    br.sendTransform( (inp[0],inp[1],inp[2]), q, time, "base_link","world")
    # rospy.sleep(0.1)

def update_posx(msg):
    inp[0] = msg.data

def update_posy(msg):
    inp[1] = msg.data

def update_posz(msg):
    inp[2] = msg.data

def update_torx(msg):
    inp[3] = msg.data

def update_tory(msg):
    inp[4] = msg.data

def update_torz(msg):
    inp[5] = msg.data


if __name__=="__main__":
    br = TransformBroadcaster()
    rospy.init_node("moving_node")
    frameCallback()
