#!/usr/bin/env python
import rospy
import math
from pyproj import Proj
from R3_functions import quat_rot, fit_in_rad, debug_info

import tf
import geometry_msgs.msg

# ROS messages
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Control Class
class waypoint_control(object):
  def __init__(self):
    # State
    self.STOP = 0
    # waypoint/state
    self.RUNNING = 1
    self.PARK = 2
    # waypoint/robot_state
    self.TURNING = 1
    self.FORWARDING = 2
    self.IDLE = 3
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
    self.projection = Proj(proj="utm", zone="34", ellps='WGS84')
    self.goal_set = False
    self.pose_get = False
    self.orentation_get = False
    self.distance = 0.0
    self.z_dist = 0.0
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0
    self.goal = Point()
    self.vel = Twist()
    self.robot_pose = Pose()
    
    # Init ROS node
    rospy.init_node('waypoint_control')
    self.freq = 10  # 10 Hz
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
    rospy.Subscriber('waypoint/flying_thres', Float32, self.flyThresCB)
    
  def Start(self):
    while not rospy.is_shutdown():
      # Wait until waypoint is set
      if not self.goal_set:
        continue
      # When the waypoint is activated
      if self.state == self.RUNNING:  
        # When the robot is turning
        if self.robot_state == self.TURNING:
          # Stop linear movements
          self.vel.linear.x = 0
          self.vel.linear.y = 0
          self.vel.linear.z = 0
          # Turn if the robot has DOF
          if self.rx_config:
            self.vel.angular.x = self.Accelerate(self.vel.angular.x, self.K_ROLL * self.roll, self.ACC_R/self.freq)
          else:
            self.vel.angular.x = 0
          if self.ry_config:
            self.vel.angular.y = self.Accelerate(self.vel.angular.y, self.K_PITCH * self.pitch, self.ACC_R/self.freq)
          else:
            self.vel.angular.y = 0
          if self.rz_config:
            self.vel.angular.z = self.Accelerate(self.vel.angular.z, self.K_YAW * self.yaw, self.ACC_R/self.freq)
          else:
            self.vel.angular.z = 0
          # Check whether turning process is finished
          finished_turning = True
          if self.rx_config and math.fabs(self.roll) > self.TURNING_THRES:  
            finished_turning = False
          if self.ry_config and math.fabs(self.pitch) > self.TURNING_THRES:  
            finished_turning = False
          if self.rz_config and math.fabs(self.yaw) > self.TURNING_THRES:  
            finished_turning = False
          # If turning is finished          
          if finished_turning:
            self.vel.angular.x = 0
            self.vel.angular.y = 0
            self.vel.angular.z = 0
            self.robot_state = self.FORWARDING     
        # When the robot is moving forwarding    
        elif self.robot_state == self.FORWARDING:
          # TODO: movement in x-y plane should be optimised
          self.vel.linear.x = self.Accelerate(self.vel.linear.x, self.K_RHO * self.distance, self.ACC_R/self.freq)
          # If the robot is able to fly
          if self.z_config:
            self.vel.linear.z = self.Accelerate(self.vel.linear.z, self.K_RHO * self.z_dist, self.ACC_R/self.freq)
          # Correct the orenitation if the robot can
          if self.rx_config:
            self.vel.angular.x = self.Accelerate(self.vel.angular.x, self.K_ROLL * self.roll, self.ACC_R/self.freq)
          else:
            self.vel.angular.x = 0
          if self.ry_config:
            self.vel.angular.y = self.Accelerate(self.vel.angular.y, self.K_PITCH * self.pitch, self.ACC_R/self.freq)
          else:
            self.vel.angular.y = 0
          if self.rz_config:
            self.vel.angular.z = self.Accelerate(self.vel.angular.z, self.K_YAW * self.yaw, self.ACC_R/self.freq)
          else:
            self.vel.angular.z = 0
          
          # Check whether the waypoint is reached
          finished_forwarding = True
          if math.fabs(self.distance) > self.FORWARDING_THRES:
            finished_forwarding = False
          if self.z_config and math.fabs(self.z_dist) > self.FLYING_THRES:
            finished_forwarding = False
          # When reach the waypoint, stop the robot and wait for new command
          if finished_forwarding:
            self.StopRobot()
          
          # If the orientation off too much, enter TURNING mode
          turning_needed = False
          if self.rx_config and math.fabs(self.roll) > math.pi/4:  
            turning_needed = True
          if self.ry_config and math.fabs(self.pitch) > math.pi/4:  
            turning_needed = True
          if self.rz_config and math.fabs(self.yaw) > math.pi/4:  
            turning_needed = True
          if turning_needed:
            self.robot_state = self.TURNING
        
        else:
          # If waypoint/state and waypoint is set, robot should run
          if self.goal_set:
            self.robot_state = self.TURNING
        
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
  
  # ROS callback function
  def stateCB(self, s):
    if s.data == "RUNNING":
      self.state = self.RUNNING
    elif s.data == "PARK":
      self.state = self.PARK
    else:  
      self.state = self.STOP
    
  def poseCB(self, p):
    self.robot_pose = p.pose.pose
    self.robot_pose.position.x, self.robot_pose.position.y = self.projection(p.pose.pose.position.x, p.pose.pose.position.y)
    self.robot_pose.orientation.x = -p.pose.pose.orientation.x
    self.robot_pose.orientation.y = -p.pose.pose.orientation.y
    self.robot_pose.orientation.z = -p.pose.pose.orientation.z
    self.robot_pose.orientation = quat_rot(self.robot_pose.orientation, 0, 0, 90)
    self.pose_get = True
    self.orentation_get = True
    if not self.goal_set:
      return
    self.distance = math.sqrt( (self.goal.x-self.robot_pose.position.x)**2 + (self.goal.y-self.robot_pose.position.y)**2 )
    self.z_dist = self.robot_pose.position.z - self.goal.z
    robot_euler = tf.transformations.euler_from_quaternion((self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w))
    self.roll = 0
    self.pitch = math.atan2(self.goal.z-self.robot_pose.position.z, math.sqrt((self.goal.x-self.robot_pose.position.x)**2 + (self.goal.y-self.robot_pose.position.y)**2)) - robot_euler[1]
    self.yaw = math.atan2(self.goal.y-self.robot_pose.position.y, self.goal.x-self.robot_pose.position.x) - robot_euler[2]
    self.roll = fit_in_rad(self.roll)
    self.pitch = fit_in_rad(self.pitch)
    self.yaw = fit_in_rad(self.yaw)
  
  def goalCB(self, g):
    x,y = self.projection(g.longitude, g.latitude)
    z = g.altitude
    if not self.pose_get:
      self.robot_pose.position.x = x
      self.robot_pose.position.y = y
      self.robot_pose.position.z = z
      self.pose_get = True
      return
    self.goal.x = x
    self.goal.y = y
    self.goal.z = z
    self.goal_set = True 
    debug_info(self.debug_output, "Waypoint received")
    if self.orentation_get:
      return
    dx = x - self.robot_pose.position.x
    dy = y - self.robot_pose.position.y
    dz = z - self.robot_pose.position.z
    robot_roll = 0
    robot_pitch = math.atan2(dz,math.sqrt(dx**2+dy**2))
    robot_yaw = math.atan2(dy,dx)
    robot_quat = tf.transformations.quaternion_from_euler(robot_roll, robot_pitch, robot_yaw)
    self.robot_pose.orientation.x = robot_quat[0]
    self.robot_pose.orientation.y = robot_quat[1]
    self.robot_pose.orientation.z = robot_quat[2]
    self.robot_pose.orientation.w = robot_quat[3]
    self.orentation_get = True
    # Publish robot initial position to calibrate the Odometry
    robot_gps_pose = Odometry()
    robot_gps_pose.pose.pose = self.robot_pose
    robot_gps_pose.pose.pose.position.x,robot_gps_pose.pose.pose.position.y = self.projection(self.robot_pose.position.x, self.robot_pose.position.y, inverse=True)
    robot_gps_pose.pose.pose.orientation.x = -self.robot_pose.orientation.x
    robot_gps_pose.pose.pose.orientation.y = -self.robot_pose.orientation.y
    robot_gps_pose.pose.pose.orientation.z = -self.robot_pose.orientation.z
    robot_gps_pose.pose.pose.orientation = quat_rot(robot_gps_pose.pose.pose.orientation,0,0,90)
    self.robot_gps_pub.publish(robot_gps_pose)
    
  def paraCB(self, p):
    if len(p.data) <= 0:
      debug_info(self.debug_output, "waypoint_control parameter invalid") 
      return
    parts = p.data.split(',')
    if len(parts) != 4:
      debug_info(self.debug_output, "waypoint_control error: 4 parameters needed, only " + str(len(parts)) + " sent") 
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
  ctrl = waypoint_control() 
  ctrl.Start()

