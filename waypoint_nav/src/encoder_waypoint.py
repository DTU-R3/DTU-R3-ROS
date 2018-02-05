#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Point, Pose2D, Twist
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

# Variables
global k_rho, k_alpha, state, substate, goal_set, distance, angle, goal, vel, vel_maxlin, vel_maxang, robot_x, robot_y, robot_th
k_rho = 0.3
k_alpha = 0.8
state = "STOP"
robot_state = "STOP"
goal_set = False
distance = 0.0
angle = math.pi
goal = Point()
vel = Twist()
vel.linear.x = 0
vel.angular.z = 0
vel_maxlin = 1
vel_maxang = 2

# Callback functions
def goalCB(g):
  global goal_set, goal, substate
  goal.x = g.x * math.cos(robot_th) - g.y * math.sin(robot.th) + robot_x
  goal.y = g.x * math.sin(robot_th) + g.y * math.cos(robot.th) + robot_y
  goal_set = True
  robot_state = "STOP"
  print "Waypoint received"

def paraCB(p):
  global k_rho, k_alpha
  if len(p.data) > 0:
    parts = p.data.split(',')
    if len(parts) == 2:
      k_rho = float(parts[0])
      k_alpha = float(parts[1])
      print "Parameter updated:"
      print "k_rho: " + str(k_rho)
      print "k_alpha: " + str(k_alpha) 
    else:
      print "2 parameter needed, only " + str(len(parts)) + " sent"

def stateCB(s):
  global state
  state = s.data
  print "Waypoint control state updated: " + state
  
def odomCB(odo):
  global goal_set, distance, angle, goal
  robot_x = odo.pose.pose.position.x
  robot_y = odo.pose.pose.position.y
  robot_th = 2 * math.atan2(odo.pose.pose.orientation.z,odo.pose.pose.orientation.w)
  if goal_set:
    dx = (goal.x-robot_x)*math.cos(robot_th)+(goal.y-robot_y)*math.sin(robot_th)
    dy = -(goal.x-robot_x)*math.sin(robot_th)+(goal.y-robot_y)*math.cos(robot_th)
    distance = math.sqrt( (dx)**2 + (dy)**2 )
    angle = math.atan2(dy,dx)

def linCB(l):
  global vel_maxlin
  vel_maxlin = l.data
  print "Max linear speed is set to: " + str(vel_maxlin)

def angCB(a):
  global vel_maxang
  vel_maxang = a.data
  print "Max angular speed is set to: " + str(vel_maxang)  

# Init ROS node
rospy.init_node('waypoint_control')

# Publishers
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
state_pub = rospy.Publisher('waypoint/robot_state', String, queue_size = 10)

# Subscribers
state_sub = rospy.Subscriber('waypoint/state', String, stateCB)
odom_sub = rospy.Subscriber('odom', Odometry, odomCB)
goal_sub = rospy.Subscriber('waypoint/goal', Point, goalCB)
para_sub = rospy.Subscriber('waypoint/control_parameters', String, paraCB)
maxlin_sub = rospy.Subscriber('waypoint/max_linear_speed', Float32, linCB)
maxang_sub = rospy.Subscriber('waypoint/max_angular_speed', Float32, angCB)

rate = rospy.Rate(100)

while not rospy.is_shutdown():

  if goal_set:
      if state == "RUNNING":
        if robot_state != "FORWARDING":
          robot_state = "TURNING"    
    
        if robot_state == "TURNING":
          vel.linear.x = 0
          vel.angular.z = k_alpha * angle
          if math.fabs(angle) < 0.2:
            robot_state = "FORWARDING"        
        elif robot_state == "FORWARDING":
      	  if (angle > math.pi/2):
            vel.linear.x = -k_rho * distance
            vel.angular.z = k_alpha * (angle-math.pi)  
          elif (angle < -math.pi/2):
            vel.linear.x = -k_rho * distance
            vel.angular.z = k_alpha * (angle+math.pi)  
          else:
            vel.linear.x = k_rho * distance
            vel.angular.z = k_alpha * angle 
    
        if vel.linear.x > vel_maxlin:
          vel.linear.x = vel_maxlin
        if vel.angular.z > vel_maxang:
          vel.angular.z = vel_maxang
     
        vel_pub.publish(vel)
      
      elif state == "PARK":
        robot_state = "STOP"
        vel.linear.x = 0
        vel.angular.z = 0
        vel_pub.publish(vel)
      
      else:
        if prestate == "RUNNING":
          robot_state = "STOP"
          vel.linear.x = 0
          vel.angular.z = 0
          vel_pub.publish(vel)
        else:
          substate = "IDLE"
          
      timecount = 0
      pose_received = False

    prestate = state
        
  state_pub.publish(substate)
  rate.sleep()

