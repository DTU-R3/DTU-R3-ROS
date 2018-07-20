#!/usr/bin/env python 
import rospy 
import sys
import csv 
import math
import datetime

from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
 
# Class 
class padbot_u1(object): 
  def __init__(self): 
    rospy.init_node('padbot_u1')
    self.rate = rospy.Rate(10)
    
    self.x = 0
    self.y = 0
    self.theta = 0
    self.v = 0
    self.w = 0
    self.left_speed = 0
    self.right_speed = 0
    self.left_count = 0
    self.right_count = 0
    self.last_left = 0
    self.last_right = 0
    self.time = rospy.Time.now()   
    
    # Padbot parameters
    self.track_width = 0.228
    self.distance_per_count = 0.0000439
    
    # Publlishers
    self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
    self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=10)
    self.resetPub= rospy.Publisher('padbot/reset_encoder', Bool, queue_size = 10)
    
    # Subcribers    
    rospy.Subscriber('padbot/left_speed', Float32, self.leftSpdCB)  
    rospy.Subscriber('padbot/right_speed', Float32, self.rightSpdCB)
    rospy.Subscriber('padbot/left_count', Int32, self.leftCB)  
    rospy.Subscriber('padbot/right_count', Int32, self.rightCB)
      
  def Running(self):
    reset_msg = Bool()
    reset_msg.data = True
    self.resetPub.publish(reset_msg)
    rospy.sleep(1)
    
    self.odom.header.frame_id = "odom"
    self.odom.child_frame_id = "base_footprint"
    self.odom.pose.pose.position.z = 0
    while not rospy.is_shutdown():
      # Calculate robot postion
      deltaLeft = self.left_count - self.last_left
      deltaRight = self.right_count - self.last_right
      delta_time = float(str(rospy.Time.now() - self.time))/1000000000
      self.last_left = self.left_count
      self.last_right = self.right_count
      if math.fabs(deltaLeft) > 10000 or math.fabs(deltaRight) > 10000:
        continue
      if self.left_count == 0 and self.right_count ==0:
        if math.fabs(deltaLeft) > 0 or math.fabs(deltaRight) > 0:
          continue
      deltaDistance = (deltaLeft + deltaRight) * self.distance_per_count / 2
      deltaAngle = (deltaRight - deltaLeft) * self.distance_per_count / self.track_width
      self.x += deltaDistance * math.cos(self.pose.theta)
      self.y += deltaDistance * math.sin(self.pose.theta)
      self.theta += deltaAngle
      self.v = (self.left_speed + self.right_speed) / 2
      self.w = (self.right_speed - self.left_speed) / self.track_width
      
      # Publish odometry
      quaternion = Quaternion()
      quaternion.x = 0.0
      quaternion.y = 0.0
      quaternion.z = sin(self.theta / 2.0)
      quaternion.w = cos(self.theta / 2.0)
      ros_now = rospy.Time.now()
      self._OdometryTransformBroadcaster.sendTransform(
        (self.x, self.y, 0),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        ros_now,
        "base_footprint",
        "odom"
      )
      odometry = Odometry()
      odometry.header.frame_id = "odom"
      odometry.header.stamp = ros_now
      odometry.pose.pose.position.x = self.x
      odometry.pose.pose.position.y = self.y
      odometry.pose.pose.position.z = 0
      odometry.pose.pose.orientation = quaternion
      odometry.child_frame_id = "base_link"
      odometry.twist.twist.linear.x = self.v
      odometry.twist.twist.linear.y = 0
      odometry.twist.twist.angular.z = self.w
      self._OdometryPublisher.publish(odometry)
    
    # Set the frame and position for the odometry
    odometry = odo
    odometry.header.frame_id = "odom"
    odometry.header.stamp = ros_now
    odometry.child_frame_id = "zed"
    odometry.pose.pose.position.z = 0
      self.rate.sleep()
  
  def leftSpdCB(self, l):
    self.left_speed = l.data
    
  def rightSpdCB(self, r):
    self.right_speed = r.data
    
  def leftCB(self, l):
    self.left_count = l.data
    
  def rightCB(self, r):
    self.right_count = r.data

if __name__ == '__main__':  
  robot = padbot_u1()
  try:
    robot.Running()
  except rospy.ROSInterruptException:
    pass
    
