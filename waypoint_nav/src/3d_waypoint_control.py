#!/usr/bin/env python

import rospy
import math
import tf

from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Float32

# Variables
global k_rho, k_roll, k_pitch, k_yaw, state, substate, goal_set, distance, roll, pitch, yaw, goal, vel, vel_maxlin, vel_maxang, pose_received
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
vel.linear.y = 0
vel.linear.z = 0
vel.angular.x = 0
vel.angular.y = 0
vel.angular.z = 0
vel_maxlin = 1
vel_maxang = 2
pose_received = False
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
      k_roll = float(parts[1])
      k_pitch = float(parts[2])
      k_yaw = float(parts[3])
      print "Parameter updated:"
      print "k_rho: " + str(k_rho)
      print "k_roll: " + str(k_roll) 
      print "k_pitch: " + str(k_pitch)
      print "k_yaw: " + str(k_yaw) 
    else:
      print "2 parameter needed, only " + str(len(parts)) + " sent"

def stateCB(s):
  global state
  state = s.data
  print "Waypoint control state updated: " + state

def quaternion_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)
    
def quaternion_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z
      
def poseCB(p):
  global goal_set, distance, roll, pitch, yaw, goal, pose_received
  pose_received = True
  q = (p.pose.pose.orientation.w, p.pose.pose.orientation.x, p.pose.pose.orientation.y, p.pose.pose.orientation.z)
  q_conjugate = quaternion_conjugate(q)
  if goal_set:
    w =(0, goal.x, goal.y, goal.z)
    w_result = quaternion_mult(quaternion_mult(q, w), q_conjugate)
    distance = math.sqrt( (goal.x-p.pose.pose.position.x)**2 + (goal.y-p.pose.pose.position.y)**2 + (goal.z-p.pose.pose.position.z)**2 )
    euler = tf.transformations.euler_from_quaternion(w_result)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    print "Distance: "+str(distance) 
    print "Roll: "+str(roll)
    print "Pitch: "+str(pitch)
    print "Yaw: "+str(yaw)

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
pose_sub = rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, poseCB)
goal_sub = rospy.Subscriber('waypoint/goal', Point, goalCB)
para_sub = rospy.Subscriber('waypoint/control_parameters', String, paraCB)
maxlin_sub = rospy.Subscriber('waypoint/max_linear_speed', Float32, linCB)
maxang_sub = rospy.Subscriber('waypoint/max_angular_speed', Float32, angCB)

rate = rospy.Rate(100)
timeout = 100;
timecount = 0;

while not rospy.is_shutdown():

  if goal_set:
    if pose_received == True:
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
      	  if (yaw > math.pi/2):
            vel.linear.x = -k_rho * distance
            vel.angular.x = k_roll * roll
            vel.angular.y = k_pitch * pitch
            vel.angular.z = k_yaw * (yaw-math.pi)  
          elif (yaw < -math.pi/2):
            vel.linear.x = -k_rho * distance
            vel.angular.x = k_roll * roll
            vel.angular.y = k_pitch * pitch
            vel.angular.z = k_yaw * (yaw+math.pi)  
          else:
            vel.linear.x = k_rho * distance
            vel.angular.x = k_roll * roll
            vel.angular.y = k_pitch * pitch
            vel.angular.z = k_yaw * yaw 
    
        if vel.linear.x > vel_maxlin:
          vel.linear.x = vel_maxlin
        if vel.angular.x > vel_maxang:
          vel.angular.x = vel_maxang
        if vel.angular.y > vel_maxang:
          vel.angular.y = vel_maxang
        if vel.angular.z > vel_maxang:
          vel.angular.z = vel_maxang
     
        vel_pub.publish(vel)
      
      elif state == "PARK":
        substate = "STOP"
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        vel_pub.publish(vel)
      
      else:
        if prestate == "RUNNING":
          substate = "STOP"
          vel.linear.x = 0
          vel.linear.y = 0
          vel.linear.z = 0
          vel.angular.x = 0
          vel.angular.y = 0
          vel.angular.z = 0
          vel_pub.publish(vel)
        else:
          substate = "IDLE"
          
      timecount = 0
      pose_received = False
    
    else:
      timecount = timecount + 1
      if timecount > timeout:
        vel.linear.x = 0
        vel.angular.z = 0
        vel_pub.publish(vel)
        state = "STOP"
        
    prestate = state
        
  state_pub.publish(substate)
  rate.sleep()

