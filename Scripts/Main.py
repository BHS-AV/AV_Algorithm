import Controls
import rospy
import numpy as np
import Navigation as nav
import Lidar as Lidar
from nav_msgs.msg import Odometry as odom
from sensor_msgs.msg import LaserScan

print "run?"
def run():
    print ("yes")


if __name__ == "__main__":
    print ("go")
    rospy.init_node('keyop')  # vesc/ackermann_cmd_mux/input/navigation ackermann_msgs/AckermannDriveStamped
    main=rospy.Subscriber('main', LaserScan,run , queue_size=2)
    pub = rospy.Subscriber('odom', odom, nav.print_odata, queue_size=2)
    lidarsub = rospy.Subscriber('scan', LaserScan, Lidar.print_data, queue_size=2)

while not rospy.is_shutdown():
    pass