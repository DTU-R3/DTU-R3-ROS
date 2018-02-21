#!/usr/bin/env python

import rospy
import math
import json
from pyproj import Proj
from pyquaternion import Quaternion

# TF libraries
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

# ROS messages
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialMapEntryArray, FiducialMapEntry, FiducialTransformArray

# Variables
global projection, tfBuffer, listener, gps_fiducials, reference_id
projection = Proj(proj="utm", zone="34", ellps='WGS84')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
fiducials_gps = FiducialMapEntryArray()
reference_id = 0
robot_gps_pose = Odometry()

# Math functions
def degToRad(d):
  r = d * math.pi / 180.0
  return r

# ROS Callback functions
def mapGPSCB(GPS_map):
  global fiducials_gps
  fiducials_gps = GPS_map

def transCB(t):
  global reference_id
  reference_id = t.transforms[-1].fiducial_id

# Init ROS node
rospy.init_node('fiducial_waypoint_localization')

# rosparams
global robot_frame, fiducial_frame
robot_frame = rospy.get_param("waypoint_control/base_frame", "base_footprint")
fiducial_map_file = rospy.get_param("waypoint_control/map_file", "/home/ros/catkin_ws/src/DTU-R3-ROS/waypoint_nav/src/Fiducials.json")
        
# Publishers
robot_gps_pub = rospy.Publisher('robot_gps_pose', Odometry, queue_size = 10)
tf_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)

# Subscribers
map_gps_sub = rospy.Subscriber('fiducial_map_GPS', FiducialMapEntryArray, mapGPSCB)
detect_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, transCB)

rate = rospy.Rate(100)

# Init fiducials map in GPS from json
json_data = json.load(open(fiducial_map_file))
for fid in json_data["FiducialCollections"][0]["SavedFiducials"]:
  fid_gps_map = FiducialMapEntry()
  fid_gps_map.fiducial_id = fid["Id"]
  fid_gps_map.x = fid["Position"]["longitude"]
  fid_gps_map.y = fid["Position"]["latitude"]
  fid_gps_map.z = fid["Position"]["altitude"]
  fid_gps_map.rx = fid["Rotation"]["x"]
  fid_gps_map.ry = fid["Rotation"]["y"]
  fid_gps_map.rz = fid["Rotation"]["z"]
  fiducial_gps_map.fiducials.append(fid_gps_map)
fiducials_gps = fiducial_gps_map

while not rospy.is_shutdown():

  # Find the transfrom from robot to utm_fiducial 
  for fid in fiducials_gps.fiducials:
    if fid.fiducial_id == reference_id:	
      fid_utm_x, fid_utm_y = projection(fid.x, fid.y) 
      # Forward transform
      t_fwd = geometry_msgs.msg.TransformStamped()
      t_fwd.header.frame_id = "utm_fiducial_test"
      t_fwd.header.stamp = rospy.Time.now()   
      t_fwd.child_frame_id = "fid_fiducial_test"
      quat = tf.transformations.quaternion_from_euler(degToRad(fid.rx), degToRad(fid.ry), degToRad(fid.rz))
      t_fwd.transform.translation.x = fid_utm_x
      t_fwd.transform.translation.y = fid_utm_y
      t_fwd.transform.translation.z = fid.z
      t_fwd.transform.rotation.x = quat[0]
      t_fwd.transform.rotation.y = quat[1]
      t_fwd.transform.rotation.z = quat[2]
      t_fwd.transform.rotation.w = quat[3]
      tfm_fwd = tf2_msgs.msg.TFMessage([t_fwd])
      tf_pub.publish(tfm_fwd)
      
      # Inverse transform
      try:
        trans = tfBuffer.lookup_transform("fid_fiducial_test", "utm_fiducial_test", rospy.Time())
	t = geometry_msgs.msg.TransformStamped()
	t.header.frame_id = "fid"+str(fid.fiducial_id)
	t.header.stamp = rospy.Time.now()   
	t.child_frame_id = "utm_fiducial"
	t.transform = trans.transform
	tfm = tf2_msgs.msg.TFMessage([t])
	tf_pub.publish(tfm)
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
	print "Can not find the transformation from utm_fiducial to fiducials"

      
  # Calculate and publish robot gps position
  try:
    trans = tfBuffer.lookup_transform("utm_fiducial", robot_frame, rospy.Time())
    robot_gps_pose.pose.pose.position.x, robot_gps_pose.pose.pose.position.y = projection(trans.transform.translation.x, trans.transform.translation.y, inverse=True)
    robot_gps_pose.pose.pose.position.z = trans.transform.translation.z
    robot_gps_pose.pose.pose.orientation = trans.transform.rotation
    robot_gps_pub.publish(robot_gps_pose)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print "Can not find the transformation from " + robot_frame + " to utm_fiducial"
  
  rate.sleep()
  
