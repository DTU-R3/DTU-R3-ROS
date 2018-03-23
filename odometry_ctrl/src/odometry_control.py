#!/usr/bin/env python

import rospy
import math

import tf

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

# STATEs
global STOP, RUNNING, TURNING, FORWARDING, IDLE, ARRIVED, state, robot_state, prestate
STOP = 0
# odometry_control/state
RUNNING = 1
PARK = 2
# odometry_control/robot_state
TURNING = 1
FORWARDING = 2
IDLE = 3
ARRIVED = 4
state = STOP
robot_state = STOP
prestate = STOP

# Control parameters
global K_RHO, K_ALPHA, FORWARDING_THRES, TURNING_THRES, VEL_MAX_LIN, VEL_MAX_ANG, ACC, ACC_R
FORWARDING_THRES = 0.1
TURNING_THRES = 0.2
VEL_MAX_LIN = 1.0
VEL_MAX_ANG = 1.0
K_RHO = 0.3
K_ALPHA = 0.8
ACC = 0.2
ACC_R = 0.2

# Variables
global robot_x, robot_y, robot_th, target_x, target_y, target_th, vel
robot_x = 0.0
robot_y = 0.0
robot_th = 0.0
target_x = 0.0
target_y = 0.0
target_th = 0.0
vel = Twist()
distance = 0.0
angle = 0.0

# Control functions
def LimitRange(v, l):
  if v > 0:
    return min(v, math.fabs(l))
  else:
    return max(v, -math.fabs(l))

def StopRobot():
  global robot_state, vel
  robot_state = STOP
  vel.linear.x = 0
  vel.angular.x = 0
  vel.angular.y = 0
  vel.angular.z = 0
  vel_pub.publish(vel)
  
def fitInRad(r):
  while r > math.pi:
    r = r - 2 * math.pi
  while r < -math.pi:
    r = r + 2 * math.pi
  return r

def Accelerate(v, cmd_v, acc):
  if v - cmd_v > acc:
    vel = v - acc
  elif cmd_v - v > acc:
    vel = v + acc
  else:
    vel = cmd_v
  return vel
  
# Callback functions
def odomCB(odo):
  global robot_x, robot_y, robot_th
  robot_x = odo.pose.pose.position.x
  robot_y = odo.pose.pose.position.y
  robot_euler = tf.transformations.euler_from_quaternion((odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, odo.pose.pose.orientation.z, odo.pose.pose.orientation.w))
  robot_th = robot_euler[2] # 2 * math.atan2(odo.pose.pose.orientation.z,odo.pose.pose.orientation.w)
  
def stateCB(s):
  global state
  if s.data == "RUNNING":
    state = RUNNING
  elif s.data == "PARK":
    state = PARK
  else:  
    state = STOP
  print "Odometry control state updated: " + s.data

def cmdCB(cmd):
  global robot_state, target_x, target_y, target_th, robot_x, robot_y, robot_th
  global FORWARDING, TURNING 
  cmd_parts = cmd.data.split(',')  
  s = cmd_parts[0] 
  if s == "fwd":
    dis = float(cmd_parts[1])
    target_x = robot_x + dis * math.cos(robot_th)
    target_y = robot_y + dis * math.sin(robot_th) 
    robot_state = FORWARDING
  elif s == "turn":  
    a = float(cmd_parts[1]) * math.pi / 180.0
    target_th = robot_th + a
    target_th = fitInRad(target_th)
    robot_state = TURNING

def paraCB(p):
  global K_RHO, K_ALPHA
  if len(p.data) > 0:
    parts = p.data.split(',')
    if len(parts) == 2:
      K_RHO = float(parts[0])
      K_ALPHA = float(parts[1])
      print "Parameter updated: " + str(K_RHO) +", " + str(K_ALPHA)
    else:
      print "Error: 2 parameter needed, only " + str(len(parts)) + " sent"

def accCB(a):
  global ACC, ACC_R
  if len(a.data) > 0:
    parts = p.data.split(',')
    if len(parts) == 2:
      ACC = float(parts[0])
      ACC_R = float(parts[1])
      print "Acceleration updated: " + str(ACC) +", " + str(ACC_R)
    else:
      print "Error: 2 parameter needed, only " + str(len(parts)) + " sent"

def linCB(l):
  global VEL_MAX_LIN
  VEL_MAX_LIN = l.data
  print "Max linear speed is set to: " + str(VEL_MAX_LIN)

def angCB(a):
  global VEL_MAX_ANG
  VEL_MAX_ANG = a.data
  print "Max angular speed is set to: " + str(VEL_MAX_ANG)  

def fwdThresCB(thres):
  global FORWARDING_THRES
  FORWARDING_THRES = thres.data
  print "Forwarding threshold is set to: " + str(FORWARDING_THRES)
    
def trunThresCB(thres):
  global TURINING_THRES
  TURNING_THRES = thres.data
  print "Turning threshold is set to: " + str(TURNING_THRES)

# Init ROS node
rospy.init_node('odometry_control')

# Publishers
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
robot_state_pub = rospy.Publisher('odometry_control/robot_state', String, queue_size = 10)

# Subscribers
odom_sub = rospy.Subscriber('odom', Odometry, odomCB)
cmd_sub = rospy.Subscriber('odometry_control/cmd', String, cmdCB)
state_sub = rospy.Subscriber('odometry_control/state', String, stateCB)
para_sub = rospy.Subscriber('odometry_control/control_parameters', String, paraCB)
acc_sub = rospy.Subscriber('odometry_control/acceleration', String, accCB)
maxlin_sub = rospy.Subscriber('odometry_control/max_linear_speed', Float32, linCB)
maxang_sub = rospy.Subscriber('odometry_control/max_angular_speed', Float32, angCB)
fwding_thres_sub = rospy.Subscriber('odometry_control/forwarding_thres', Float32, fwdThresCB)
turning_thres_sub = rospy.Subscriber('odometry_control/turning_thres', Float32, trunThresCB)

freq = 10
rate = rospy.Rate(freq)

while not rospy.is_shutdown():
  if state == RUNNING: 
    if robot_state == TURNING:     
      angle = fitInRad(target_th - robot_th)
      vel.linear.x = 0
      vel.angular.z = Accelerate(vel.angular.z, K_ALPHA * angle, ACC_R/freq)
      if math.fabs(angle) < TURNING_THRES:
        vel.linear.x = 0
        vel.angular.z = 0
        robot_state = ARRIVED     
    elif robot_state == FORWARDING:
      distance = math.sqrt( (target_x-robot_x)**2 + (target_y-robot_y)**2)
      angle = fitInRad(math.atan2(target_y-robot_y,target_x-robot_x)-robot_th)
      if math.fabs(distance) > FORWARDING_THRES:
        vel.linear.x = Accelerate(vel.linear.x, K_RHO * distance, ACC/freq)
        vel.angular.z = Accelerate(vel.angular.z, K_ALPHA * angle, ACC_R/freq)
      else:
        vel.linear.x = 0
        vel.angular.z = 0
        robot_state = ARRIVED 
    
    print "distance: " + str(distance)  
    print "angle: " + str(angle)  
    vel.linear.x = LimitRange(vel.linear.x, VEL_MAX_LIN)
    vel.angular.x = LimitRange(vel.angular.x, VEL_MAX_ANG)
    vel.angular.y = LimitRange(vel.angular.y, VEL_MAX_ANG)
    vel.angular.z = LimitRange(vel.angular.z, VEL_MAX_ANG)
    vel_pub.publish(vel)
    
  elif state == PARK:
    StopRobot()
      
  else:
    if prestate == RUNNING:
      StopRobot()
    else:
      robot_state = IDLE      

  prestate = state       
  robot_state_pub.publish(str(robot_state))
  rate.sleep()

