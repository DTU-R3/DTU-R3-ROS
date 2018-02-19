#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Float32
from pyquaternion import Quaternion

# Variables
global k_rho, k_roll, k_pitch, k_yaw, state, substate, goal_set, distance, roll, pitch, yaw, goal, vel, vel_maxlin, vel_maxang, prestate
k_rho = 0.3
k_roll = 0.8
k_pitch = 0.8
k_yaw = 0.8
state = "STOP"
substate = "STOP"
goal_set = False
distance = 0.0
roll = math.pi
pitch = math.pi
yaw = math.pi
goal = Point()
vel = Twist()
vel.linear.x = 0
vel.angular.z = 0
vel_maxlin = 1
vel_maxang = 2
prestate = "STOP"

# Callback functions
def goalCB(g):
  global goal_set, goal, substate
  goal = g
  goal_set = True
  substate = "STOP"
  print "Waypoint received"

def paraCB(p):
  global k_rho, k_roll, k_pitch, k_yaw
  if len(p.data) > 0:
    parts = p.data.split(',')
    if len(parts) == 4:
      k_rho = float(parts[0])
      k_alpha = float(parts[1])
      k_pitch = float(parts[2])
      k_yaw = float(parts[3])
      print "Parameter updated:"
      print "k_rho: " + str(k_rho)
      print "k_roll: " + str(k_roll) 
      print "k_pitch: " + str(k_pitch)
      print "k_yaw: " + str(k_yaw) 
    else:
      print "4 parameter needed, only " + str(len(parts)) + " sent"

def stateCB(s):
  global state
  state = s.data
  print "Waypoint control state updated: " + state
  
def poseCB(p):
  global goal_set, distance, roll, pitch, yaw, goal
  if goal_set:
    distance = math.sqrt( (goal.x-p.pose.pose.position.x)**2 + (goal.y-p.pose.pose.position.y)**2 + (goal.z-p.pose.pose.position.z)**2 )
    q_rot = Quaternion(p.pose.pose.orientation.w, p.pose.pose.orientation.x, p.pose.pose.orientation.y, p.pose.pose.orientation.z)
    x,y,z=q_rot.rotate([(goal.x-p.pose.pose.position.x),(goal.y-p.pose.pose.position.y),(goal.z-p.pose.pose.position.z)])
    if distance != 0:
      roll = 0
      pitch = asin(z/distance)
      yaw = atan2(y,x)
    else:
      roll = 0
      pitch = 0
      yaw = 0

def linCB(l):
  global vel_maxlin
  vel_maxlin = l.data
  print "Max linear speed is set to: " + str(vel_maxlin)

def angCB(a):
  global vel_maxang
  vel_maxang = a.data
  print "Max angular speed is set to: " + str(vel_maxang)  

# Init ROS node
rospy.init_node('3D_waypoint_control')

# Publishers
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
state_pub = rospy.Publisher('waypoint/robot_state', String, queue_size = 10)

# Subscribers
state_sub = rospy.Subscriber('waypoint/state', String, stateCB)
pose_sub = rospy.Subscriber('fiducial_pose', PoseWithCovarianceStamped, poseCB)
goal_sub = rospy.Subscriber('waypoint/goal', Point, goalCB)
para_sub = rospy.Subscriber('waypoint/control_parameters', String, paraCB)
maxlin_sub = rospy.Subscriber('waypoint/max_linear_speed', Float32, linCB)
maxang_sub = rospy.Subscriber('waypoint/max_angular_speed', Float32, angCB)

rate = rospy.Rate(100)

while not rospy.is_shutdown():

  if goal_set:
    if state == "RUNNING":
      if substate != "FORWARDING":
        substate = "TURNING"    
    
      if substate == "TURNING":
        vel.linear.x = 0
        vel.angular.x = k_roll * roll
        vel.angular.y = k_pitch * pitch
        vel.angular.z = k_yaw * yaw
        if math.fabs(yaw) < 0.2:
          substate = "FORWARDING"        
      elif substate == "FORWARDING":
      	if (angle > math.pi/2):
          vel.linear.x = -k_rho * distance
          vel.angular.z = k_yaw * (yaw-math.pi) 
        elif (angle < -math.pi/2):
          vel.linear.x = -k_rho * distance
          vel.angular.z = k_yaw * (yaw+math.pi)  
        else:
          vel.linear.x = k_rho * distance
          vel.angular.z = k_yaw * yaw 
    
      if vel.linear.x > vel_maxlin:
        vel.linear.x = vel_maxlin
      if vel.angular.z > vel_maxang:
        vel.angular.z = vel_maxang
     
      vel_pub.publish(vel)
      
    elif state == "PARK":
      substate = "STOP"
      vel.linear.x = 0
      vel.angular.z = 0
      vel_pub.publish(vel)
      
    else:
      if prestate == "RUNNING":
        substate = "STOP"
        vel.linear.x = 0
        vel.angular.z = 0
        vel_pub.publish(vel)
      else:
        substate = "IDLE"
        
    prestate = state
        
  state_pub.publish(substate)
  rate.sleep()

