#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

global linear_vel, angular_vel, state, prestate, robot_state, distance, angle
global robot_x, robot_y, robot_th, start_x, start_y, start_th, vel, vel_maxlin, vel_maxang

linear_vel = 0.5
angular_vel = 0.5
state = "STOP"
robot_state = "idle"
distance = 0.0
angle = 0.0
dis_thres = 0.2
ang_thres = 0.2
prestate = "STOP"
robot_x = 0.0
robot_y = 0.0
robot_th = 0.0
start_x = 0.0
start_y = 0.0
start_th = 0.0
vel = Twist()
vel_maxlin = 1
vel_maxang = 2

def fitInRadians(d):
  r = d * math.pi / 180
  while (r > math.pi):
    r = r - 2 * math.pi
  while (r < -math.pi):
    r = r + 2 * math.pi
  return r
  
# Callback functions
def odomCB(odo):
  global robot_x, robot_y, robot_th
  robot_x = odo.pose.pose.position.x
  robot_y = odo.pose.pose.position.y
  robot_th = 2 * math.atan2(odo.pose.pose.orientation.z,odo.pose.pose.orientation.w)

def stateCB(s):
  global state
  state = s.data

def cmdCB(cmd):
  global robot_state, distance, angle, linear_vel, angular_vel, robot_x, robot_y, robot_th, start_x, start_y, start_th
  cmd_parts = cmd.data.split(',')  
  robot_state = cmd_parts[0] 
  if robot_state == "forward":
    distance = float(cmd_parts[1])
    if len(cmd_parts) > 2:
      linear_vel = float(cmd_parts[2])
    start_x = robot_x
    start_y = robot_y
    start_th = robot_th     
  elif robot_state == "turn":
    angle = fitInRadians(float(cmd_parts[1]))
    if len(cmd_parts) > 2:
      angular_vel = float(cmd_parts[2])
    start_x = robot_x
    start_y = robot_y
    start_th = robot_th
      
# Init ROS node
rospy.init_node('odometry_control')

# Publishers
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
state_pub = rospy.Publisher('odometry_control/robot_state', String, queue_size = 10)

# Subscribers
odom_sub = rospy.Subscriber('odom', Odometry, odomCB)
state_sub = rospy.Subscriber('odometry_control/state', String, stateCB)
cmd_sub = rospy.Subscriber('odometry_control/cmd', String, cmdCB)

rate = rospy.Rate(1000)

while not rospy.is_shutdown():
  if state == "RUNNING":
    if robot_state == "forward":
      if distance > 0:
        vel.linear.x = linear_vel
      else:
        vel.linear.x = -linear_vel
      vel.angular.z = 0
      if ( math.fabs(math.sqrt( (robot_x-start_x)**2 + (robot_y-start_y)**2 ) - math.fabs(distance)) < dis_thres):
        robot_state = "stop"
        
    elif robot_state == "turn":
      vel.linear.x = 0
      if angle > 0:
        vel.angular.z = angular_vel
      else:
        vel.angular.z = -angular_vel
      if (math.fabs( math.fabs(robot_th - start_th) - math.fabs(angle)) < ang_thres):
        robot_state = "stop"
        
    elif robot_state == "stop":
      vel.linear.x = 0
      vel.angular.z = 0
      
    if vel.linear.x > vel_maxlin:
      vel.linear.x = vel_maxlin
    if vel.angular.z > vel_maxang:
      vel.angular.z = vel_maxang   
    vel_pub.publish(vel)
       
  else:
    if prestate == "RUNNING":
      robot_state = "stop"
      vel.linear.x = 0
      vel.angular.z = 0
      vel_pub.publish(vel)
    else:
      robot_state = "idle"
  
  prestate = state
  state_pub.publish(robot_state)
  rate.sleep()

