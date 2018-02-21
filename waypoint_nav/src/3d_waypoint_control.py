#!/usr/bin/env python

import rospy
import math
from pyproj import Proj
from pyquaternion import Quaternion

# ROS messages
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix



# STATEs
global STOP, RUNNING, TURNING, FORWARDING, IDLE, state, robot_state, prestate
STOP = 0
# waypoint/state
RUNNING = 1
PARK = 2
# waypoint/robot_state
TURNING = 1
FORWARDING = 2
IDLE = 3
state = STOP
robot_state = STOP
prestate = STOP

# Control parameters
global TURNING_THRES, K_RHO, K_ROLL, K_PITCH, K_YAW, VEL_MAX_LIN, VEL_MAX_ANG
TURNING_THRES = 0.2
VEL_MAX_LIN = 1.0
VEL_MAX_ANG = 2.0
K_RHO = 0.3
K_ROLL = 0.8
K_PITCH = 0.8
K_YAW = 0.8

# Variables
global projection, goal_set, pose_get, orentation_get
global distance, roll, pitch, yaw, goal, vel, robot_pose
projection = Proj(proj="utm", zone="34", ellps='WGS84')
goal_set = False
pose_get = False
orentation_get = False
distance = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0
goal = Point()
vel = Twist()
robot_pose = Pose()

# Math functions
def DegtoRad(d):
  r = d * math.pi / 180.0
  return r
  
def LimitRange(v, l):
  if v > 0:
    return min(v,math.abs(l))
  else:
    return max(v,math.abs(l))

def StopRobot():
  global robot_state, vel
  robot_state = STOP
  vel.linear.x = 0
  vel.angular.x = 0
  vel.angular.y = 0
  vel.angular.z = 0
  vel_pub.publish(vel)
    
# ROS Callback functions
def paraCB(p):
  global K_RHO, K_ROLL, K_PITCH, K_YAW
  if len(p.data) > 0:
    parts = p.data.split(',')
    if len(parts) == 4:
      K_RHO = float(parts[0])
      K_ROLL = float(parts[1])
      K_PITCH = float(parts[2])
      K_YAW = float(parts[3])
      print "Parameter updated: " + str(K_RHO) +", " + str(K_ROLL) +", " + str(K_PITCH) +", " + str(K_YAW)
    else:
      print "Error: 4 parameter needed, only " + str(len(parts)) + " sent"

def stateCB(s):
  global state
  if s.data == "RUNNING":
    state = RUNNING
  elif s.data == "PARK":
    state = PARK
  else:  
    state = STOP
  print "Waypoint control state updated: " + s.data

def linCB(l):
  global VEL_MAX_LIN
  VEL_MAX_LIN = l.data
  print "Max linear speed is set to: " + str(VEL_MAX_LIN)

def angCB(a):
  global VEL_MAX_ANG
  VEL_MAX_ANG = a.data
  print "Max angular speed is set to: " + str(VEL_MAX_ANG)  
  
def thresCB(thres):
  global TURINING_THRES
  TURNING_THRES = thres.data
  print "Turning threshold is set to: " + str(TURNING_THRES)

def goalCB(g):
  global projection, goal_set, goal, robot_state, pose_get, orentation_get, robot_pose
  x,y = projection(g.longitude, g.latitude)
  z = g.altitude
  if not pose_get:
    robot_pose.position.x = x
    robot_pose.position.y = y
    robot_pose.position.z = z
    pose_get = True
  else:  
    if not orentation_get:
      dx = x - robot_pose.position.x
      dy = y - robot_pose.position.y
      dz = z - robot_pose.position.z
      robot_roll = 0
      robot_pitch = math.atan2(dz,math.sqrt(dx**2+dy**2))
      robot_yaw = math.atan2(dy,dx)
      robot_quat = tf.transformations.quaternion_from_euler(robot_roll, robot_pitch, robot_yaw)
      robot_pose.orientation.x = quat[0]
      robot_pose.orientation.y = quat[1]
      robot_pose.orientation.z = quat[2]
      robot_pose.orientation.w = quat[3]
      orentation_get = True
    goal.x = x
    goal.y = y
    goal.z = z
    goal_set = True 
  robot_state = STOP
  print "Waypoint received"
  
def poseCB(p):
  global goal_set, distance, roll, pitch, yaw, goal, pose_get, orentation_get
  robot_pose = p.pose.pose
  robot_pose.position.x, robot_pose.position.y = projection(p.pose.pose.position.x, p.pose.pose.position.y)
  pose_get = True
  orentation_get = True
  if goal_set:
    distance = math.sqrt( (goal.x-robot_pose.position.x)**2 + (goal.y-robot_pose.position.y)**2 + (goal.z-robot_pose.position.z)**2 )
    q_rot = Quaternion(robot_pose.orientation.w, robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z)
    x,y,z=q_rot.rotate([(goal.x-robot_pose.position.x),(goal.y-robot_pose.position.y),(goal.z-robot_pose.position.z)])
    if distance != 0:
      roll = 0
      pitch = math.atan2(z, math.sqrt(x**2+y**2))
      yaw = math.atan2(y,x)
    else:
      roll = 0
      pitch = 0
      yaw = 0
      
# Init ROS node
rospy.init_node('3D_waypoint_control')

# Publishers
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
robot_state_pub = rospy.Publisher('waypoint/robot_state', String, queue_size = 10)

# Subscribers
state_sub = rospy.Subscriber('waypoint/state', String, stateCB)
pose_sub = rospy.Subscriber('robot_gps_pose', Odometry, poseCB)
goal_sub = rospy.Subscriber('waypoint', NavSatFix, goalCB)
para_sub = rospy.Subscriber('waypoint/control_parameters', String, paraCB)
maxlin_sub = rospy.Subscriber('waypoint/max_linear_speed', Float32, linCB)
maxang_sub = rospy.Subscriber('waypoint/max_angular_speed', Float32, angCB)
turning_thres_sub = rospy.Subscriber('waypoint/turning_thres', Float32, thresCB)

rate = rospy.Rate(100)

while not rospy.is_shutdown():

  if goal_set:
    if state == RUNNING:
      if robot_state != FORWARDING:
        robot_state = TURNING   
    
      if robot_state == TURNING:
        vel.linear.x = 0
        vel.angular.x = K_ROLL * roll
        vel.angular.y = K_PITCH * pitch
        vel.angular.z = K_YAW * yaw
        if math.fabs(yaw) < TURNING_THRES:
          robot_state = FORWARDING     
      elif robot_state == FORWARDING:
      	if (yaw > math.pi/2):
          vel.linear.x = -K_RHO * distance
          vel.angular.z = K_YAW * (yaw-math.pi) 
        elif (yaw < -math.pi/2):
          vel.linear.x = -K_RHO * distance
          vel.angular.z = K_YAW * (yaw+math.pi)  
        else:
          vel.linear.x = K_RHO * distance
          vel.angular.z = K_YAW * yaw 
      
      vel.linear.x = LimitRange(vel.linear.x, VEL_MAX_LIN)
      vel.angular.x = LimitRange(vel.angular.z, VEL_MAX_ANG)
      vel.angular.y = LimitRange(vel.angular.z, VEL_MAX_ANG)
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
  robot_state_pub.publish(robot_state)
  rate.sleep()

