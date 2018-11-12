#!/usr/bin/env python
import rospy
import math
import numpy
import random
from R3_functions import fit_in_rad, debug_info

import tf

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan

# Control Class
class corridor_nav(object):
  def __init__(self):
    
    # Variables
    self.vel = Twist()
    self.scan = LaserScan()
    self.corridorMode = False
    self.scan_received = False
    self.y_left = 0 
    self.y_right = 0

    # Control parameters
    self.K = 0.8
    self.VEL_MAX_LIN = 0.5
    self.VEL_MAX_ANG = 1.0

    # Init ROS node
    rospy.init_node('corridor_nav')
    self.freq = 10
    self.rate = rospy.Rate(self.freq)
    
    # Publishers
    self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    self.debug_output = rospy.Publisher('debug_output', String, queue_size = 10)

    # Subscribers
    rospy.Subscriber('/central_mass/cmd_vel', Twist, self.velCB)
    rospy.Subscriber('/scan', LaserScan, self.scanCB)
    rospy.Subscriber('/corridor_mode', Bool, self.modeCB)
  
  def Start(self):
    while not rospy.is_shutdown():
      laser_scan = self.scan
      # If corridor mode is not enable, do nothing
      if not self.corridorMode or not self.scan_received:
        self.rate.sleep()
        continue
      
      # Calculate the central mass of left side
      left = 0
      num_left = 0
      min_left = min(laser_scan.ranges[181:270])
      for i in range(181,270):
        if not math.isinf(laser_scan.ranges[i]) and laser_scan.ranges[i] < min_left+1:
          left += laser_scan.ranges[i]*math.sin(math.radians(i-180))
          num_left += 1

      # Calculate the central mass of right side
      right = 0
      num_right = 0
      min_right = min(laser_scan.ranges[90:180])
      for i in range(90,180):
        if not math.isinf(laser_scan.ranges[i]) and laser_scan.ranges[i] < min_right+1:
          right += laser_scan.ranges[i]*math.sin(math.radians(180-i))
          num_right += 1

      self.y_left = left / num_left
      self.y_right = right / num_right
      print "---"
      print self.y_left
      print self.y_right
      self.rate.sleep()
  
  def velCB(self, v):
    if self.corridorMode:
      # Correct the vel based on central mass
      if self.y_left == 0 and self.y_right == 0:
        self.vel_pub.publish(v)
        return
      self.vel = v
      if self.y_left == 0:
        self.vel.linear.x = 0
        self.vel.angular.z = -0.2
      elif self.y_right == 0:
        self.vel.linear.x = 0
        self.vel.angular.z = 0.2
      else:
        self.vel.angular.z += self.K/(self.y_right**2) - self.K/(self.y_left**2)
      self.vel.linear.x = self.LimitRange(self.vel.linear.x, self.VEL_MAX_LIN)
      self.vel.angular.z = self.LimitRange(self.vel.angular.z, self.VEL_MAX_ANG)
      self.vel_pub.publish(self.vel)
    else:
      self.vel_pub.publish(v)
  
  def scanCB(self, s):
    if self.corridorMode:
      self.scan = s 
      self.scan_received = True

  def modeCB(self, b):
    self.corridorMode = b.data
  
  def LimitRange(self, v, l):
    if v > 0:
      return min(v, math.fabs(l))
    else:
      return max(v, -math.fabs(l))
	  
if __name__ == '__main__': 
  nav = corridor_nav() 
  nav.Start()  

