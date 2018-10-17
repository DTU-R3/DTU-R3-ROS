#!/usr/bin/env python
import rospy
import math
import tf
from pyproj import Proj
from R3_functions import quat_rot, fit_in_rad, debug_info

import tf
import geometry_msgs.msg

# ROS messages
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

# Control Class
class waypoint_prediction(object):
  def __init__(self):       
    # Variables
    self.dt = 3    # Predict pose in 3 seconds
    self.vel = Twist()
    self.robot_pose = Pose()
    self.robot_prediction = Odometry()
    self.vel_received = False
    self.pose_received = False
    self.projection = Proj(proj="utm", zone="34", ellps='WGS84')

    # Init ROS node
    rospy.init_node('waypoint_prediction')
    self.freq = 1  # 1 Hz
    self.rate = rospy.Rate(self.freq)	
    
    # Publishers
    self.pred_pub = rospy.Publisher('robot_predict_pose', Odometry, queue_size = 10)

    # Subscribers
    rospy.Subscriber('cmd_vel', Twist, self.velCB)
    rospy.Subscriber('robot_gps_pose', Odometry, self.poseCB)
    
  def Start(self):
    while not rospy.is_shutdown():
      if not self.vel_received:
        self.rate.sleep()
        continue
      if not self.pose_received:
        self.rate.sleep()
        continue
      pose = self.robot_pose
      pose.position.x, pose.position.y = self.projection(self.robot_pose.position.x, self.robot_pose.position.y)
      pose.orientation.x = -self.robot_pose.orientation.x
      pose.orientation.y = -self.robot_pose.orientation.y
      pose.orientation.z = -self.robot_pose.orientation.z
      pose.orientation = quat_rot(pose.orientation, 0, 0, 90)
      v = self.vel.linear.x
      w = self.vel.angular.z
      alpha = w * self.dt / 2
      euler = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
      theta = euler[2]
      if w==0:
        d = v * self.dt
      else:
        r = v / w
        d = 2 * r * math.sin(alpha)
      pose.position.x += d * math.cos(alpha + theta)
      pose.position.y += d * math.sin(alpha + theta)     
      pose.orientation = quat_rot(pose.orientation,0,0, math.degrees(alpha))
      pose.orientation.x = -pose.orientation.x
      pose.orientation.y = -pose.orientation.y
      pose.orientation.z = -pose.orientation.z   
      pose.orientation = quat_rot(pose.orientation,0,0,90)
      self.robot_prediction.pose.pose = pose
      self.robot_prediction.pose.pose.position.x,self.robot_prediction.pose.pose.position.y = self.projection(pose.position.x,pose.position.y,inverse=True)
      self.pred_pub.publish(self.robot_prediction)
      self.vel_received = False
      self.rate.sleep()
  
  # Control functions
  def velCB(self, v):
    self.vel = v
    self.vel_received = True
    
  def poseCB(self, p):
    self.robot_pose = p.pose.pose
    self.pose_received = True

if __name__ == '__main__': 
  pred = waypoint_prediction() 
  pred.Start()

