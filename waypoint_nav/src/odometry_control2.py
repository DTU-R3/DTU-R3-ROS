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
    self.TURNING = 1
    self.FORWARDING = 2
    self.IDLE = 3

    # State variables
    self.robot_state = self.STOP

    # Control parameters
    self.FORWARDING_THRES = 0.2
    self.TURNING_THRES = 0.2
    self.VEL_LIN = 0.5
    self.VEL_ANG = 1.0
    
    # Variables
    self.robot_pos = Pose()
    self.target_pos = Pose()
    self.vel = Twist()
    self.finished = Bool()
    	
    # Init ROS node
    rospy.init_node('odometry_control2')
    self.freq = 50
    self.rate = rospy.Rate(self.freq)

    # Publishers
    self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    self.finished_pub = rospy.Publisher('odometry_control/finished', Bool, queue_size = 10)

    # Subscribers
    rospy.Subscriber('odom', Odometry, self.odomCB)
    rospy.Subscriber('odometry_control/cmd', String, self.cmdCB)
    rospy.Subscriber('odometry_control/linear_speed', Float32, self.linCB)
    rospy.Subscriber('odometry_control/angular_speed', Float32, self.angCB)
    rospy.Subscriber('odometry_control/forwarding_thres', Float32, self.fwdThresCB)
    rospy.Subscriber('odometry_control/turning_thres', Float32, self.trunThresCB)
    
  def Start(self):
    while not rospy.is_shutdown():
      if self.finished.data:
        self.rate.sleep()
        continue

      # Calculate relative position
      pos = self.robot_pos
      distance = math.sqrt( (self.target_pos.position.x-pos.position.x)**2 + (self.target_pos.position.y-pos.position.y)**2 )
      robot_euler = tf.transformations.euler_from_quaternion((pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w))
      target_euler = tf.transformations.euler_from_quaternion((self.target_pos.orientation.x, self.target_pos.orientation.y, self.target_pos.orientation.z, self.target_pos.orientation.w))
      angle = fit_in_rad( math.atan2((self.target_pos.position.y-pos.position.y),(self.target_pos.position.x-pos.position.x)) - robot_euler[2])
      if math.fabs(angle) < math.pi/2:
        fwd_dir = 1.0
      else:
        fwd_dir = -1.0
      yaw = fit_in_rad(target_euler[2] - robot_euler[2])
      if yaw > 0:
        turn_dir = 1.0
      else:
        turn_dir = -1.0
      print "distance: " + str(distance)
      print "yaw: " + str(yaw)
      # When the robot is turning
      if self.robot_state == self.TURNING:
        # Stop linear movements
        self.vel.linear.x = 0
        self.vel.angular.z = turn_dir * self.VEL_ANG

        # Check whether turning process is finished
        finished_turning = True
        if math.fabs(yaw) > self.TURNING_THRES:  
          finished_turning = False

        # If turning is finished  
        if finished_turning:
          if not self.finished.data:
            self.finished.data = True
            self.finished_pub.publish(self.finished)
            self.StopRobot()
            self.robot_state = self.IDLE
        
      # When the robot is moving forwarding    
      elif self.robot_state == self.FORWARDING:       
        self.vel.linear.x = fwd_dir * self.VEL_LIN
        self.vel.angular.z = 0
          
        # Check whether the target is reached
        finished_forwarding = True
        if math.fabs(distance) > self.FORWARDING_THRES:
          finished_forwarding = False

        # When reach the target, stop the robot and wait for new command
        if finished_forwarding:
          if not self.finished.data:
            self.finished.data = True
            self.finished_pub.publish(self.finished)
            self.StopRobot()
            self.robot_state = self.IDLE

      # When the cmd is not known
      else:
        self.robot_state = self.IDLE
        self.StopRobot()
        self.finished.data = True
        self.finished_pub.publish(self.finished)

      self.vel_pub.publish(self.vel)
      self.rate.sleep()
	  
  # Control functions
  def StopRobot(self):
    self.vel.linear.x = 0
    self.vel.angular.z = 0
    self.vel_pub.publish(self.vel)
    self.finished.data = True
  
  # Callback functions
  def odomCB(self, odo):
    self.robot_pos = odo.pose.pose
  
  def cmdCB(self, cmd):
    cmd_parts = cmd.data.split(',')  
    s = cmd_parts[0]
    pos = self.robot_pos
    robot_euler = tf.transformations.euler_from_quaternion((pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w))
    robot_th = robot_euler[2]
    if s == "fwd":
      dis = float(cmd_parts[1])
      self.target_pos.position.x = pos.position.x + dis * math.cos(robot_th)
      self.target_pos.position.y = pos.position.y + dis * math.sin(robot_th) 
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
    else:
      self.robot_state = self.STOP
      self.finished.data = False
  
  def linCB(self, l):
    self.VEL_LIN = l.data
    print "Linear speed is set to: " + str(self.VEL_LIN)

  def angCB(self, a):
    self.VEL_ANG = a.data
    print "Angular speed is set to: " + str(self.VEL_ANG)

  def fwdThresCB(self, thres):
    self.FORWARDING_THRES = thres.data
    print "Forwarding threshold is set to: " + str(self.FORWARDING_THRES)
    
  def trunThresCB(self, thres):
    self.TURNING_THRES = thres.data
    print "Turning threshold is set to: " + str(self.TURNING_THRES)
  
if __name__ == '__main__': 
  ctrl = odometry_control() 
  ctrl.Start()  

