#!/usr/bin/env python

import rospy
import math
import json
from pyproj import Proj
from R3_functions import quat_rot, fit_in_rad, debug_info

# TF libraries
import tf
import tf2_ros
import tf2_msgs.msg

# ROS messages
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

# Class
class fiducial_mapping(object):
  def __init__(self):
    # Init ROS node
    rospy.init_node('fiducial_mapping')    

    # Variables
    self.projection = Proj(proj="utm", zone="34", ellps='WGS84')
    self.tfBuffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
    self.fid_id = 0

    # rosparams
    self.robot_frame = rospy.get_param("~waypoint_control/base_frame", "base_footprint")
    self.gps_frame = rospy.get_param("~waypoint_control/gps_frame", "utm")
    self.camera_frame = rospy.get_param("~waypoint_control/camera_frame", "raspicam") 

    # Subscribers
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transCB)

    # 1 Hz
    self.rate = rospy.Rate(1)
      
  def Start(self):
    while not rospy.is_shutdown():
      try:
        fid_utm_trans = self.tfBuffer.lookup_transform(self.gps_frame, str(self.fid_id), rospy.Time())
        x,y = self.projection(fid_utm_trans.transform.translation.x, fid_utm_trans.transform.translation.y, inverse=True)
        z = fid_utm_trans.transform.translation.z
        euler = tf.transformations.euler_from_quaternion((-fid_utm_trans.transform.rotation.x, -fid_utm_trans.transform.rotation.y, -fid_utm_trans.transform.rotation.z, fid_utm_trans.transform.rotation.w))
        print "---"
        print x
        print y
        print z
        print math.degrees(euler[0])
        print math.degrees(euler[1])
        print math.degrees(euler[2])+90
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):    
        continue
      self.rate.sleep()
    
  def transCB(self, t):
    for fid_trans in t.transforms:
      self.fid_id = fid_trans.fiducial_id

if __name__ == '__main__': 
  fid = fiducial_mapping() 
  fid.Start()
  
