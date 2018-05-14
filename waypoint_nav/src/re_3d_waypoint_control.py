#!/usr/bin/env python
import rospy
import math
from pyproj import Proj
from waypoint_nav.srv import *

import tf
import geometry_msgs.msg

# ROS messages
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Class
class waypoint_control(object):
  def __init__(self):
    # Init ROS node
    rospy.init_node('waypoint_control')
    freq = 10  # 10 Hz
    rate = rospy.Rate(freq)	
    
    # Parameters
    self.x_config = rospy.get_param("~robot_x_config", True)
    self.y_config = rospy.get_param("~robot_y_config", True)
    self.z_config = rospy.get_param("~robot_z_config", False)
    self.rx_config = rospy.get_param("~robot_rx_config", False)
    self.ry_config = rospy.get_param("~robot_ry_config", False)
    self.rz_config = rospy.get_param("~robot_rz_config", True)
    
    # Publishers
    self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    self.robot_state_pub = rospy.Publisher('waypoint/robot_state', String, queue_size = 10)
    self.robot_gps_pub = rospy.Publisher('odo_calib_pose', Odometry, queue_size = 10)
    self.debug_output = rospy.Publisher('debug_output', String, queue_size = 10)

    # Subscribers
    rospy.Subscriber('waypoint/state', String, self.stateCB)
    rospy.Subscriber('robot_gps_pose', Odometry, self.poseCB)
    rospy.Subscriber('waypoint', NavSatFix, self.goalCB)
    rospy.Subscriber('waypoint/control_parameters', String, self.paraCB)
    rospy.Subscriber('waypoint/acceleration', String, self.accCB)
    rospy.Subscriber('waypoint/max_linear_speed', Float32, self.linCB)
    rospy.Subscriber('waypoint/max_angular_speed', Float32, self.angCB)
    rospy.Subscriber('waypoint/forwarding_thres', Float32, self.fwdThresCB)
    rospy.Subscriber('waypoint/turning_thres', Float32, self.trunThresCB)
    
    # State
    self.STOP = 0
    # waypoint/state
    self.RUNNING = 1
    self.PARK = 2
    # waypoint/robot_state
    self.TURNING = 1
    self.FORWARDING = 2
    self.IDLE = 3
    self.ARRIVED = 4
    # State variables
    self.state = STOP
    self.robot_state = STOP
    self.prestate = STOP
    
    # Control parameters
    self.FORWARDING_THRES = 0.1
    self.FLYING_THRES = 1.0
    self.TURNING_THRES = 0.2
    self.VEL_MAX_LIN = 0.5
    self.VEL_MAX_ANG = 1.0
    self.K_RHO = 0.3
    self.K_ROLL = 0.8
    self.K_PITCH = 0.8
    self.K_YAW = 0.8
    self.ACC = 0.1
    self.ACC_R = 0.1
    
    # Variables
    self.projection = Proj(proj="utm", zone="34", ellps='WGS84')
    self.goal_set = False
    self.pose_get = False
    self.orentation_get = False
    self.distance = 0.0
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0
    self.goal = Point()
    self.vel = Twist()
    self.robot_pose = Pose()
    
  def Start(self):
    while not rospy.is_shutdown():
      if not self.goal_set:
        continue
      
      if state == RUNNING:    
        if robot_state == TURNING:
          vel.linear.x = 0
          vel.angular.x = Accelerate(vel.angular.x, K_ROLL * roll, ACC_R/freq)
          vel.angular.y = Accelerate(vel.angular.y, K_PITCH * pitch, ACC_R/freq)
          vel.angular.z = Accelerate(vel.angular.z, K_YAW * yaw, ACC_R/freq)
        if math.fabs(yaw) < TURNING_THRES:
          vel.angular.x = 0
          vel.angular.y = 0
          vel.angular.z = 0
          robot_state = FORWARDING     
        elif robot_state == FORWARDING:
          if math.fabs(distance) > FORWARDING_THRES:
      	    vel.linear.x = Accelerate(vel.linear.x, K_RHO * distance, ACC/freq)
      	    vel.angular.y = Accelerate(vel.angular.y, K_PITCH * pitch, ACC_R/freq)
            vel.angular.z = Accelerate(vel.angular.z, K_YAW * yaw, ACC_R/freq)
            if math.fabs(yaw) > math.pi/4:
               robot_state = TURNING
            else:
              vel.linear.x = 0
      	      vel.angular.y = 0
              vel.angular.z = 0
              robot_state = ARRIVED
      
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
  
  # Control functions
  def StopRobot(self):
    self.robot_state = STOP
    self.vel.linear.x = 0
    self.vel.linear.y = 0
    self.vel.linear.z = 0
    self.vel.angular.x = 0
    self.vel.angular.y = 0
    self.vel.angular.z = 0
    self.vel_pub.publish(vel)
    
  def LimitRange(self, v, l):
    if v > 0:
      return min(v, math.fabs(l))
    else:
      return max(v, -math.fabs(l))
  
  def Accelerate(self, v, cmd_v, acc):
    if v - cmd_v > acc:
      vel = v - acc
    elif cmd_v - v > acc:
      vel = v + acc
    else:
      vel = cmd_v
    return vel
  
  # Servica client
  def FitInRad_client(r):
    rospy.wait_for_service('FitInRad')
    try:
      fit_in_rad = rospy.ServiceProxy('FitInRad', FitInRad)
      resp = fit_in_rad(r)
      return resp.rad
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
  
  def FitInRad_client(q, deg_x, deg_y, deg_z):
    rospy.wait_for_service('QuatRot')
    try:
      quat_rot = rospy.ServiceProxy('QuatRot', FitInRad)
      resp = quat_rot(r)
      return resp.quat
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
  
  # ROS callback function
  

if __name__ == '__main__': 
  ctrl = waypoint_control() 
  waypoint_control.Start() 

########### ----------------------------------- #############

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
      
def stateCB(s):
  global state, RUNNING, PARK, STOP
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

def fwdThresCB(thres):
  global FORWARDING_THRES
  FORWARDING_THRES = thres.data
  print "Forwarding threshold is set to: " + str(FORWARDING_THRES)
    
def trunThresCB(thres):
  global TURINING_THRES
  TURNING_THRES = thres.data
  print "Turning threshold is set to: " + str(TURNING_THRES)

def goalCB(g):
  global projection, goal_set, goal, robot_state, pose_get, orentation_get, robot_pose
  global STOP
  x,y = projection(g.longitude, g.latitude)
  z = g.altitude
  if not pose_get:
    robot_pose.position.x = x
    robot_pose.position.y = y
    robot_pose.position.z = z
    pose_get = True
  else:  
    goal.x = x
    goal.y = y
    goal.z = z
    goal_set = True 
    print "Waypoint received"
    if not orentation_get:
      dx = x - robot_pose.position.x
      dy = y - robot_pose.position.y
      dz = z - robot_pose.position.z
      robot_roll = 0
      robot_pitch = math.atan2(dz,math.sqrt(dx**2+dy**2))
      robot_yaw = math.atan2(dy,dx)
      robot_quat = tf.transformations.quaternion_from_euler(robot_roll, robot_pitch, robot_yaw)
      robot_pose.orientation.x = robot_quat[0]
      robot_pose.orientation.y = robot_quat[1]
      robot_pose.orientation.z = robot_quat[2]
      robot_pose.orientation.w = robot_quat[3]
      orentation_get = True
      # Publish robot initial position
      robot_gps_pose = Odometry()
      robot_gps_pose.pose.pose = robot_pose
      robot_gps_pose.pose.pose.position.x,robot_gps_pose.pose.pose.position.y = projection(robot_pose.position.x, robot_pose.position.y, inverse=True)
      robot_gps_pose.pose.pose.orientation.x = -robot_pose.orientation.x
      robot_gps_pose.pose.pose.orientation.y = -robot_pose.orientation.y
      robot_gps_pose.pose.pose.orientation.z = -robot_pose.orientation.z
      robot_gps_pose.pose.pose.orientation = quatRot(robot_gps_pose.pose.pose.orientation,0,0,90)
      robot_gps_pub.publish(robot_gps_pose)  
  robot_state = STOP
  
def poseCB(p):
  global goal_set, distance, roll, pitch, yaw, goal, pose_get, orentation_get
  robot_pose = p.pose.pose
  robot_pose.position.x, robot_pose.position.y = projection(p.pose.pose.position.x, p.pose.pose.position.y)
  robot_pose.orientation.x = -p.pose.pose.orientation.x
  robot_pose.orientation.y = -p.pose.pose.orientation.y
  robot_pose.orientation.z = -p.pose.pose.orientation.z
  robot_pose.orientation = quatRot(robot_pose.orientation,0,0,90)
  pose_get = True
  orentation_get = True
  if goal_set:
    distance = math.sqrt( (goal.x-robot_pose.position.x)**2 + (goal.y-robot_pose.position.y)**2 + (goal.z-robot_pose.position.z)**2 )
    robot_euler = tf.transformations.euler_from_quaternion((robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w))
    roll = 0
    pitch = math.atan2(goal.z-robot_pose.position.z, math.sqrt((goal.x-robot_pose.position.x)**2 + (goal.y-robot_pose.position.y)**2)) - robot_euler[1]
    yaw = math.atan2(goal.y-robot_pose.position.y, goal.x-robot_pose.position.x) - robot_euler[2]
    roll = fitInRad(roll)
    pitch = fitInRad(pitch)
    yaw = fitInRad(yaw) 
      





