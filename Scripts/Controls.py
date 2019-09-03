#!/usr/bin/env python
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
import Map as m
import sys, select, termios, tty

turn = 1

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)


if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
  rospy.init_node('keyop') #vesc/ackermann_cmd_mux/input/navigation ackermann_msgs/AckermannDriveStamped
  x = 0
  th = 0
  status = 0 #vesc/ackermann_cmd_mux/input/navigation ackermann_msgs/AckermannDriveStamped

def move(x1, th1, speed=1,delta=0):
    settings = termios.tcgetattr(sys.stdin)
    #pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
    rospy.init_node('keyop')  # vesc/ackermann_cmd_mux/input/navigation ackermann_msgs/AckermannDriveStamped
    pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
    #print(delta)
    try:
        m.move(x1*speed*delta*50)
        #m.move(x1*speed)
        msg = AckermannDriveStamped();
        msg.header.stamp = rospy.Time.now();
        msg.header.frame_id = "base_link";
        msg.drive.speed = x1 * speed;
        msg.drive.acceleration = .1;
        msg.drive.jerk = 1;
        msg.drive.steering_angle = th1 * turn
        msg.drive.steering_angle_velocity = 1
        #print ("moving this dude " , speed, "*",x1," ",msg.drive.speed)
        pub.publish(msg)
    except:
        print ('error')

