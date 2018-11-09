#!/usr/bin/env python
import rospy
import math
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
    
    # Control parameters
    self.K_RHO = 0.3
    self.K_YAW = 0.8
    self.ACC = 0.1
    self.ACC_R = 0.1
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
    rospy.Subscriber('/scan', LaserScan, self.scanCB)
    rospy.Subscriber('/corridor_mode', Bool, self.modeCB)
  
  def Start(self):
    while not rospy.is_shutdown():
      laser_scan = self.scan
      # If corridor mode is not enable, do nothing
      if not self.corridorMode or not self.scan_received:
        self.rate.sleep()
        continue
      # Random choose three measure from the front
      index = random.sample(range(90, 270), 3)
      # Check if any value is Inf
      inf = False
      for i in index:
        if math.isinf(laser_scan.ranges[i]):
          inf = True
      if inf:
        debug_info(self.debug_output, "Sampled Inf value from laser data")
        continue

      # Calculate the circle, relative to laser_frame
      x1,y1 = self.CalculateCoord(self.indexToRad(index[0],laser_scan.angle_min,laser_scan.angle_increment),laser_scan.ranges[index[0]])   
      x2,y2 = self.CalculateCoord(self.indexToRad(index[1],laser_scan.angle_min,laser_scan.angle_increment),laser_scan.ranges[index[1]]) 
      x3,y3 = self.CalculateCoord(self.indexToRad(index[2],laser_scan.angle_min,laser_scan.angle_increment),laser_scan.ranges[index[2]]) 
      a = 2*(x1-x2)
      b = 2*(y1-y2)
      c = (x1**2-x2**2)+(y1**2-y2**2)
      d = 2*(x2-x3)
      e = 2*(y2-y3)
      f = (x2**2-x3**2)+(y2**2-y3**2)
      flag = a*e - b*d
      if flag == 0:
        debug_info(self.debug_output, "Failed to create circle based sampled points")
        continue
      xc = (c*e-b*f)/flag
      yc = -(c*d-a*f)/flag

      # Check if the circle is qualified
      qualified = True
      i = 90
      while i < 270:
        x,y = self.CalculateCoord(self.indexToRad(i,laser_scan.angle_min,laser_scan.angle_increment),laser_scan.ranges[i])
        if ((x-xc)**2+(y-yc)**2) < ((x1-xc)**2+(y1-yc)**2):
          qualified = False
          break
        i += 1
      
      if (xc**2+yc**2) > ((x1-xc)**2+(y1-yc)**2):        
        qualified = False

      if not qualified:
        debug_info(self.debug_output, "Circle is screened out")
        continue

      # Drive to the circle
      dis = math.sqrt(xc**2+yc**2)
      yaw = fit_in_rad(math.atan2(yc, xc))
      self.vel.linear.x = self.Accelerate(self.vel.linear.x, self.K_RHO * dis, self.ACC/self.freq)
      self.vel.angular.z = self.Accelerate(self.vel.angular.z, self.K_YAW * yaw, self.ACC_R/self.freq)
      self.vel.linear.x = self.LimitRange(self.vel.linear.x, self.VEL_MAX_LIN)
      self.vel.angular.z = self.LimitRange(self.vel.angular.z, self.VEL_MAX_ANG)
      self.vel_pub.publish(self.vel)
      self.rate.sleep()
  
  def indexToRad(self, i, min_ang, ang_incre):
    rad = min_ang + i * ang_incre
    return rad

  def CalculateCoord(self, rad, dis):
    x = dis * math.cos(rad)
    y = dis * math.sin(rad)
    return x, y
  
  def Accelerate(self, v, cmd_v, acc):
    if v - cmd_v > acc:
      vel = v - acc
    elif cmd_v - v > acc:
      vel = v + acc
    else:
      vel = cmd_v
    return vel
  
  def LimitRange(self, v, l):
    if v > 0:
      return min(v, math.fabs(l))
    else:
      return max(v, -math.fabs(l))

  def scanCB(self, s):
    if self.corridorMode:
      self.scan = s 
      self.scan_received = True

  def modeCB(self, b):
    self.corridorMode = b.data
	  
if __name__ == '__main__': 
  nav = corridor_nav() 
  nav.Start()  

