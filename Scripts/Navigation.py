#!/usr/bin/env python
import Controls
import rospy
import numpy as np
import tf
from nav_msgs.msg import Odometry as odom
from geometry_msgs.msg import Quaternion


orient=[]
xorient=0
yorient=0
dir="N"
orientation=0

def get_data_array(data, a=0, b=270):
    PPD = 4 # points per degree
    data = data[a*PPD:b*PPD]
    arr = np.array(data)
    return arr



def setDir(orientation1):
    global dir
    global orientation
    orientation=orientation1
    if (orientation<45 or orientation>325):
        dir="N"
    elif (orientation<135):
        dir="E"
    elif (orientation< 225):
        dir="S"
    else:
        dir="W"

def getDir():
    global dir
    return dir

def getOrient():
    global orientation
    return orientation

def getXOrient():
    global xorient
    return xorient

def getYOrient():
    global yorient
    return yorient


def setOrient(msg):
    global orient
    # Convert quaternions to Euler angles.
    (r, p, y) = tf.transformations.euler_from_quaternion( [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    orient=[r, p, y]




def print_odata(odom):
    setOrient(odom)
    global orient
    global yorient
    global xorient
    q=odom.pose.pose.orientation
    orientation=orient[2]
    xorient=orient[0]
    yorient=orient[1]

    #print (odom.pose)
    rorient=orientation/3.14*180
    if (rorient<0):
        rorient=rorient+360
    #print ("value = ", orientation)
    #print ("orientation= ",rorient)
    #print (orient)
    setDir(rorient)