import Controls
import rospy
import Map as m
import numpy as np
import Navigation as nav
import Lidar as Lidar
import time
from nav_msgs.msg import Odometry as odom
from sensor_msgs.msg import LaserScan

def run():
    print("run method go plz?")


if __name__ == "__main__":
    print ("go")
    rospy.init_node('keyop')  # vesc/ackermann_cmd_mux/input/navigation ackermann_msgs/AckermannDriveStamped
    #mpo=rospy.Subscriber('main', LaserScan,run , queue_size=2)
    pub = rospy.Subscriber('odom', odom, nav.print_odata, queue_size=2)
    lidarsub = rospy.Subscriber('scan', LaserScan, Lidar.print_data, queue_size=2)


lrender=0
dt=0
lastsave=time.time()
imgnum=0
import os, shutil
folder = '/home/racecar/PycharmProjects/AV_Algorithm/Scripts/track'
for the_file in os.listdir(folder):
    file_path = os.path.join(folder, the_file)
    try:
        if os.path.isfile(file_path):
            os.unlink(file_path)
        #elif os.path.isdir(file_path): shutil.rmtree(file_path)
    except Exception as e:
        print(e)

while not rospy.is_shutdown():
    #m.update(Lidar.getOrient())
    if (rospy.get_time()>lrender+2.5 and Lidar.isGoingStraight()):
        st=time.time()
        lrender=rospy.get_time()
        m.render(dt)
        dt=time.time()-st
        #print ('render time : ',dt)
    if (time.time() - lastsave > 8):
        st1=time.time()
        m.saveImg(imgnum)
        lastsave = time.time()
        imgnum = imgnum + 1
        print ('img saved in ',(time.time()-st1),' sec')
    pass
