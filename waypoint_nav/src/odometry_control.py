#!/usr/bin/env python
import rospy
import math
from R3_functions import fit_in_rad, debug_info

import tf

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String, Float32, Bool
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
    self.state = self.STOP
    self.robot_state = self.STOP
    self.prestate = self.STOP

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
    self.finished = Bool()
    	
    # Init ROS node
    rospy.init_node('odometry_control')
    self.freq = 10
    self.rate = rospy.Rate(self.freq)
	
    # Parameters, robot configuration
    self.x_config = rospy.get_param("~robot_x_config", True)
    self.y_config = rospy.get_param("~robot_y_config", False)
    self.z_config = rospy.get_param("~robot_z_config", False)
    self.rx_config = rospy.get_param("~robot_rx_config", False)
    self.ry_config = rospy.get_param("~robot_ry_config", False)
    self.rz_config = rospy.get_param("~robot_rz_config", True)
    
    # Publishers
    self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    self.debug_output = rospy.Publisher('debug_output', String, queue_size = 10)
    self.finished_pub = rospy.Publisher('odometry_control/finished', Bool, queue_size = 10)

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
      if self.state == self.RUNNING: 
        # Calculate relative position
        distance = math.sqrt( (self.target_pos.position.x-self.robot_pos.position.x)**2 + (self.target_pos.position.y-self.robot_pos.position.y)**2 )
        z_dist = self.target_pos.position.y - self.robot_pos.position.y
        robot_euler = tf.transformations.euler_from_quaternion((self.robot_pos.orientation.x, self.robot_pos.orientation.y, self.robot_pos.orientation.z, self.robot_pos.orientation.w))
        target_euler = tf.transformations.euler_from_quaternion((self.target_pos.orientation.x, self.target_pos.orientation.y, self.target_pos.orientation.z, self.target_pos.orientation.w))
        angle = fit_in_rad( math.atan2((self.target_pos.position.y-self.robot_pos.position.y),(self.target_pos.position.x-self.robot_pos.position.x)) - robot_euler[2])
        if math.fabs(angle) < math.pi/2:
          fwd_dir = 1.0
        else:
          fwd_dir = -1.0
        roll = fit_in_rad(target_euler[0] - robot_euler[0]) 
        pitch = fit_in_rad(target_euler[1] - robot_euler[1]) 
        yaw = fit_in_rad(target_euler[2] - robot_euler[2])
        # When the robot is turning
        if self.robot_state == self.TURNING:
          # Stop linear movements
          self.vel.linear.x = 0
          self.vel.linear.y = 0
          self.vel.linear.z = 0
          # Turn if the robot has DOF
          if self.rx_config:
            self.vel.angular.x = self.Accelerate(self.vel.angular.x, self.K_ROLL * roll, self.ACC_R/self.freq)
          else:
            self.vel.angular.x = 0
          if self.ry_config:
            self.vel.angular.y = self.Accelerate(self.vel.angular.y, self.K_PITCH * pitch, self.ACC_R/self.freq)
          else:
            self.vel.angular.y = 0
          if self.rz_config:
            self.vel.angular.z = self.Accelerate(self.vel.angular.z, self.K_YAW * yaw, self.ACC_R/self.freq)
          else:
            self.vel.angular.z = 0
          # Check whether turning process is finished
          finished_turning = True
          if self.rx_config and math.fabs(roll) > self.TURNING_THRES:  
            finished_turning = False
          if self.ry_config and math.fabs(pitch) > self.TURNING_THRES:  
            finished_turning = False
          if self.rz_config and math.fabs(yaw) > self.TURNING_THRES:  
            finished_turning = False
          # If turning is finished          
          if finished_turning:
            self.finshed.data = True
            self.finshed_pub.publish(self.finshed)
            self.StopRobot()
            self.robot_state = self.IDLE
        
        # When the robot is moving forwarding    
        elif self.robot_state == self.FORWARDING:       
          # TODO: movement in x-y plane should be optimised
          self.vel.linear.x = self.Accelerate(self.vel.linear.x, fwd_dir * self.K_RHO * distance, self.ACC/self.freq)
          # If the robot is able to fly
          if self.z_config:
            self.vel.linear.z = self.Accelerate(self.vel.linear.z, self.K_RHO * z_dist, self.ACC_R/self.freq)
          # Correct the orenitation if the robot can
          if self.rx_config:
            self.vel.angular.x = self.Accelerate(self.vel.angular.x, self.K_ROLL * roll, self.ACC_R/self.freq)
          else:
            self.vel.angular.x = 0
          if self.ry_config:
            self.vel.angular.y = self.Accelerate(self.vel.angular.y, self.K_PITCH * pitch, self.ACC_R/self.freq)
          else:
            self.vel.angular.y = 0
          if self.rz_config:
            self.vel.angular.z = self.Accelerate(self.vel.angular.z, self.K_YAW * yaw, self.ACC_R/self.freq)
          else:
            self.vel.angular.z = 0
          
          # Check whether the target is reached
          finished_forwarding = True
          if math.fabs(distance) > self.FORWARDING_THRES:
            finished_forwarding = False
          if self.z_config and math.fabs(z_dist) > self.FLYING_THRES:
            finished_forwarding = False
          # When reach the target, stop the robot and wait for new command
          if finished_forwarding:
            self.finshed.data = True
            self.finshed_pub.publish(self.finshed)
            self.StopRobot()
            self.robot_state = self.IDLE
        
        # When the cmd is not known
        else:
          self.robot_state = self.STOP
          self.StopRobot()
        
        # Fit the velocity into the limited range    
        self.vel.linear.x = self.LimitRange(self.vel.linear.x, self.VEL_MAX_LIN)
        self.vel.linear.y = self.LimitRange(self.vel.linear.y, self.VEL_MAX_LIN)
        self.vel.linear.z = self.LimitRange(self.vel.linear.z, self.VEL_MAX_LIN)
        self.vel.angular.x = self.LimitRange(self.vel.angular.x, self.VEL_MAX_ANG)
        self.vel.angular.y = self.LimitRange(self.vel.angular.y, self.VEL_MAX_ANG)
        self.vel.angular.z = self.LimitRange(self.vel.angular.z, self.VEL_MAX_ANG)
        self.vel_pub.publish(self.vel)
            
      # If the waypoint/state is set to PARK mode
      elif self.state == self.PARK:
        self.robot_state = STOP
        self.StopRobot()
      
      # Stop waypoint control
      else:
        if self.prestate == self.RUNNING:
          self.StopRobot()
        else:
          self.robot_state = self.IDLE      
    
      self.prestate = self.state       
      self.rate.sleep()
	  
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
      self.robot_state = self.FORWARDING
      self.finished.data = False
    elif s == "turn":  
      a = math.radians( float(cmd_parts[1]) )
      target_th = robot_th + a
      target_th = fit_in_rad(target_th)
      target_quat = tf.transformations.quaternion_from_euler(robot_euler[0], robot_euler[1], target_th)
      self.target_pos.orientation.x = target_quat[0]
      self.target_pos.orientation.y = target_quat[1]
      self.target_pos.orientation.z = target_quat[2]
      self.target_pos.orientation.w = target_quat[3]
      self.robot_state = self.TURNING
      self.finished.data = False

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

