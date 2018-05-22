#!/usr/bin/env python
import rospy
import math
from R3_functions import fit_in_rad, debug_info

import tf

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry

# Control Class
class odometry_control(object):
  def __init__(self):
    # states
    self.STOP = 0
    # odometry_control/state
    self.RUNNING = 1
    self.PARK = 2
    # odometry_control/robot_state
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
    self.TURNING_THRES = 0.2
    self.FLYING_THRES = 1.0
    self.VEL_MAX_LIN = 0.5
    self.VEL_MAX_ANG = 1.0
    self.K_RHO = 0.3
    self.K_ROLL = 0.8
    self.K_PITCH = 0.8
    self.K_YAW = 0.8
    self.ACC = 0.1
    self.ACC_R = 0.1
    
    # Variables
    self.robot_pos = Pose()
    self.target_pos = Pose()
    self.vel = Twist()
    self.distance = 0.0
    self.z_dist = 0.0
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0
    	
    # Init ROS node
    rospy.init_node('odometry_control')
    self.freq = 10
    self.rate = rospy.Rate(freq)
	
    # Publishers
    self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    self.robot_state_pub = rospy.Publisher('odometry_control/robot_state', String, queue_size = 10)
	self.debug_output = rospy.Publisher('debug_output', String, queue_size = 10)

    # Subscribers
    rospy.Subscriber('odom', Odometry, self.odomCB)
    rospy.Subscriber('odometry_control/cmd', String, self.cmdCB)
    rospy.Subscriber('odometry_control/state', String, self.stateCB)
    rospy.Subscriber('odometry_control/control_parameters', String, self.paraCB)
    rospy.Subscriber('odometry_control/acceleration', String, self.accCB)
    rospy.Subscriber('odometry_control/max_linear_speed', Float32, self.linCB)
    rospy.Subscriber('odometry_control/max_angular_speed', Float32, self.angCB)
    rospy.Subscriber('odometry_control/forwarding_thres', Float32, self.fwdThresCB)
    rospy.Subscriber('odometry_control/turning_thres', Float32, self.trunThresCB)
  
  # Control functions
  def StopRobot(self):
    self.vel.linear.x = 0
    self.vel.linear.y = 0
    self.vel.linear.z = 0
    self.vel.angular.x = 0
    self.vel.angular.y = 0
    self.vel.angular.z = 0
    self.vel_pub.publish(self.vel)
    
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
  
  def Start(self):
    while not rospy.is_shutdown():
  
if __name__ == '__main__': 
  ctrl = odometry_control() 
  ctrl.Start()  
########### --------------- ################

  
def fitInRad(r):
  while r > math.pi:
    r = r - 2 * math.pi
  while r < -math.pi:
    r = r + 2 * math.pi
  return r

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

