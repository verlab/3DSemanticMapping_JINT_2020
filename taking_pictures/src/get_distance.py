#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import numpy as np

from nav_msgs.msg import Odometry

#from __future__ import print_function

odom_topic = 'odom'
travel_dist = 0.0
lastx = 0.0
lasty = 0.0
max_speed = 0.0
max_angular = 0.0

def odom_callback(data):
  global odom_topic, travel_dist, max_speed, max_angular, lastx, lasty
  x = data.pose.pose.position.x
  y = data.pose.pose.position.y
  v = data.twist.twist.linear.x
  a = data.twist.twist.angular.z 

  dx = pow(x-lastx, 2)
  dy = pow(y-lasty, 2)

  delta = np.sqrt(dx + dy)

  travel_dist += delta

  lastx = x
  lasty = y

  if v > max_speed:
    max_speed = v
  
  if a > max_angular:
    max_angular = a
  

def main(args):
  global odom_topic, travel_dist, max_speed, max_angular, lastx, lasty

  rospy.init_node('state_saver', anonymous=True)
  rate = rospy.Rate(1)

  odom_sub = rospy.Subscriber(odom_topic, Odometry, odom_callback)

  while  not rospy.is_shutdown(): 
    try:
      print("Traveled distance: %f\n" % travel_dist)
      print("Max speed: %f\n" % max_speed)
      print("Max angular: %f\n" % max_angular)
      rate.sleep()

    except KeyboardInterrupt:
      print("Keyboard exception. \n\n\n")
      break
  
if __name__ == '__main__':
  main(sys.argv)