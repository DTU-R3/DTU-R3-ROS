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
	
	# Parameters, robot configuration
    self.x_config = rospy.get_param("~robot_x_config", True)
    self.y_config = rospy.get_param("~robot_y_config", False)
    self.z_config = rospy.get_param("~robot_z_config", False)
    self.rx_config = rospy.get_param("~robot_rx_config", False)
    self.ry_config = rospy.get_param("~robot_ry_config", False)
    self.rz_config = rospy.get_param("~robot_rz_config", True)
    
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
  
    
  def Start(self):
    while not rospy.is_shutdown():
	  # TODO
	  
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
  
  # Callback functions
  def odomCB(self, odo):
    self.robot_pos = odo.pose.pose
  
  def cmdCB(self, cmd):
    cmd_parts = cmd.data.split(',')  
    s = cmd_parts[0] 
	robot_euler = tf.transformations.euler_from_quaternion((self.robot_pos.orientation.x, self.robot_pos.orientation.y, self.robot_pos.orientation.z, self.robot_pos.orientation.w))
    robot_th = robot_euler[2] # 2 * math.atan2(odo.pose.pose.orientation.z,odo.pose.pose.orientation.w)
	self.target_pos = self.robot_pos
    if s == "fwd":
      dis = float(cmd_parts[1])
      self.target_pos.position.x = self.target_pos.position.x + dis * math.cos(robot_th)
      self.target_pos.position.y = self.target_pos.position.y + dis * math.sin(robot_th) 
      self.robot_state = FORWARDING
    elif s == "turn":  
      a = math.radians( float(cmd_parts[1]) )
      target_th = robot_th + a
      target_th = fit_in_rad(target_th)
	  target_quat = tf.transformations.quaternion_from_euler(robot_euler[0], robot_euler[1], target_th)
      self.target_pos.orientation.x = target_quat[0]
      self.target_pos.orientation.y = target_quat[1]
      self.target_pos.orientation.z = target_quat[2]
      self.target_pos.orientation.w = target_quat[3]
      self.robot_state = TURNING
  
  def stateCB(self, s):
    if s.data == "RUNNING":
      self.state = self.RUNNING
    elif s.data == "PARK":
      self.state = self.PARK
    else:  
      self.state = self.STOP
  
  def paraCB(self, p):
    if len(p.data) <= 0:
      debug_info(self.debug_output, "odometry_control parameter invalid") 
      return
    parts = p.data.split(',')
    if len(parts) != 4:
      debug_info(self.debug_output, "odometry_control error: 4 parameters needed, only " + str(len(parts)) + " sent") 
      return
    try:
      self.K_RHO = float(parts[0])
      self.K_ROLL = float(parts[1])
      self.K_PITCH = float(parts[2])
      self.K_YAW = float(parts[3])
      debug_info(self.debug_output, "Parameter updated: " + str(self.K_RHO) +", " + str(self.K_ROLL) +", " + str(self.K_PITCH) +", " + str(self.K_YAW))
    except:
      return
  
  def accCB(self, a):
    if len(a.data) <= 0:
      debug_info(self.debug_output, "waypoint_control acceleration invalid") 
      return
    parts = a.data.split(',')
    if len(parts) != 2:
      debug_info(self.debug_output, "waypoint_control error: 2 accelerations needed, only " + str(len(parts)) + " sent") 
      return
    try:
      self.ACC = float(parts[0])
      self.ACC_R = float(parts[1])
      debug_info(self.debug_output, "Acceleration updated: " + str(self.ACC) +", " + str(self.ACC_R))
    except:
      return
  
  def linCB(self, l):
    self.VEL_MAX_LIN = l.data
    debug_info(self.debug_output, "Max linear speed is set to: " + str(self.VEL_MAX_LIN) )

  def angCB(self, a):
    self.VEL_MAX_ANG = a.data
    debug_info(self.debug_output, "Max angular speed is set to: " + str(self.VEL_MAX_ANG) )

  def fwdThresCB(self, thres):
    self.FORWARDING_THRES = thres.data
    debug_info(self.debug_output, "Forwarding threshold is set to: " + str(self.FORWARDING_THRES) )
    
  def trunThresCB(self, thres):
    self.TURNING_THRES = thres.data
    debug_info(self.debug_output, "Turning threshold is set to: " + str(self.TURNING_THRES) )
    
  def flyThresCB(self, thres):
    self.FLYING_THRES = thres.data
    debug_info(self.debug_output, "Flying threshold is set to: " + str(self.FLYING_THRES) )
  
if __name__ == '__main__': 
  ctrl = odometry_control() 
  ctrl.Start()  
########### --------------- ################



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

