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
dis_thres = 0.05
ang_thres = 0.05
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
  r = d
  while (r > math.pi):
    r = r - 2.0 * math.pi
  while (r < -math.pi):
    r = r + 2.0 * math.pi
  return r
  
# Callback functions
def odomCB(odo):
  global robot_x, robot_y, robot_th
  robot_x = odo.pose.pose.position.x
  robot_y = odo.pose.pose.position.y
  robot_th = 2 * math.atan2(odo.pose.pose.orientation.z,odo.pose.pose.orientation.w)

def cmdCB(cmd):
  global robot_state, distance, angle, linear_vel, angular_vel, robot_x, robot_y, robot_th, start_x, start_y, start_th
  print "Received command: " + cmd.data
  cmd_parts = cmd.data.split(',')  
  robot_state = cmd_parts[0] 
  if robot_state == "fwd":
    distance = float(cmd_parts[1])
    print "Target distance is set: " + str(distance)
    if len(cmd_parts) > 2:
      linear_vel = float(cmd_parts[2])
    start_x = robot_x
    start_y = robot_y  
  elif robot_state == "turn":
  
    angle = float(cmd_parts[1]) * math.pi / 180.0
    angle = fitInRadians(angle)
    print "Target angle is set: " + str(angle)
    if len(cmd_parts) > 2:
      angular_vel = float(cmd_parts[2])
    start_th = robot_th
  
      
# Init ROS node
rospy.init_node('odometry_control')

# Publishers
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
state_pub = rospy.Publisher('odometry_control/robot_state', String, queue_size = 10)

# Subscribers
odom_sub = rospy.Subscriber('odom', Odometry, odomCB)
cmd_sub = rospy.Subscriber('odometry_control/cmd', String, cmdCB)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
  if robot_state == "fwd":
    if ( math.fabs(math.sqrt( (robot_x-start_x)**2 + (robot_y-start_y)**2 ) - math.fabs(distance)) > dis_thres):
      if distance > 0:
        vel.linear.x = linear_vel
      else:
        vel.linear.x = -linear_vel
      vel.angular.z = 0
      print "Forwarding: " + str(math.fabs(math.sqrt( (robot_x-start_x)**2 + (robot_y-start_y)**2 ) - math.fabs(distance)))
    else:
      vel.linear.x = 0
      vel.angular.z = 0
      robot_state = "idle"
      print "Command completed!"
       
  elif robot_state == "turn":
    if (math.fabs(robot_th - fitInRadians(start_th + angle) ) > ang_thres):
      vel.linear.x = 0
      if angle > 0:
        vel.angular.z = angular_vel
      else:
        vel.angular.z = -angular_vel
      print "Turning: " + str(math.fabs(robot_th - fitInRadians(start_th + angle)))
    else:
      vel.linear.x = 0
      vel.angular.z = 0
      robot_state = "idle"
      print "Command completed!"
        
  elif robot_state == "stop":
    vel.linear.x = 0
    vel.angular.z = 0
    robot_state = "idle"
    print "Robot stopped!"
      
  if vel.linear.x > vel_maxlin:
    vel.linear.x = vel_maxlin
  if vel.angular.z > vel_maxang:
    vel.angular.z = vel_maxang
  if vel.linear.x < -vel_maxlin:
    vel.linear.x = -vel_maxlin
  if vel.angular.z < -vel_maxang:
    vel.angular.z = -vel_maxang
           
  vel_pub.publish(vel)
  state_pub.publish(robot_state) 
  rate.sleep()

