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

# Inverse transformation
def inverseTrans(f, f_child, trans):
  tf_fwd = geometry_msgs.msg.TransformStamped()
  tf_fwd.header.frame_id = "origin_child"
  tf_fwd.header.stamp = rospy.Time.now()   
  tf_fwd.child_frame_id = "origin"
  tf_fwd.transform = trans
  tfmsg_fwd = tf2_msgs.msg.TFMessage([tf_fwd])
  tf_pub.publish(tfmsg_fwd)

  try:
    inv_trans = tfBuffer.lookup_transform("origin", "origin_child", rospy.Time())
    tf_inv = geometry_msgs.msg.TransformStamped()
    tf_inv.header.frame_id = f
    tf_inv.child_frame_id = f_child
    tf_inv.header.stamp = rospy.Time.now()      
    tf_inv.transform = inv_trans.transform
    tfmsg_inv = tf2_msgs.msg.TFMessage([tf_inv])
    tf_pub.publish(tfmsg_inv) 
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print "Can not inverse the transformation for " + f + " to " + f_child

# ROS Callback functions
def mapGPSCB(GPS_map):
  global fiducials_gps
  fiducials_gps = GPS_map

def transCB(t):
  global reference_id
  reference_id = t.transforms[-1].fiducial_id
  inverseTrans(camera_frame, "fid"+str(reference_id), t.transforms[-1].transform)

# Init ROS node
rospy.init_node('fiducial_waypoint_localization')

# rosparams
global robot_frame, fiducial_frame, camera_frame
robot_frame = rospy.get_param("waypoint_control/base_frame", "base_footprint")
fiducial_map_file = rospy.get_param("waypoint_control/map_file", "/home/ros/catkin_ws/src/DTU-R3-ROS/waypoint_nav/src/Fiducials.json")
camera_frame = rospy.get_param("waypoint_control/camera_frame", "raspicam")
        
# Publishers
robot_gps_pub = rospy.Publisher('robot_gps_pose', Odometry, queue_size = 10)
tf_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)

# Subscribers
map_gps_sub = rospy.Subscriber('fiducial_map_GPS', FiducialMapEntryArray, mapGPSCB)
detect_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, transCB)

rate = rospy.Rate(1)

# Init fiducials map in GPS from json
json_data = json.load(open(fiducial_map_file))
fiducial_gps_map = FiducialMapEntryArray()
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
      t_fwd = geometry_msgs.msg.Transform\()
      quat = tf.transformations.quaternion_from_euler(degToRad(fid.rx), degToRad(fid.ry), degToRad(fid.rz))
      t_fwd.transform.translation.x = fid_utm_x
      t_fwd.transform.translation.y = fid_utm_y
      t_fwd.transform.translation.z = fid.z
      t_fwd.transform.rotation.x = quat[0]
      t_fwd.transform.rotation.y = quat[1]
      t_fwd.transform.rotation.z = quat[2]
      t_fwd.transform.rotation.w = quat[3]
      inverseTrans("fid"+str(fid.fiducial_id), "utm_fiducial", t_fwd)

  # Calculate and publish robot gps position
  try:
    trans = tfBuffer.lookup_transform("utm_fiducial", robot_frame, rospy.Time())
    robot_gps_pose.pose.pose.position.x, robot_gps_pose.pose.pose.position.y = projection(trans.transform.translation.x, trans.transform.translation.y, inverse=True)
    robot_gps_pose.pose.pose.position.z = trans.transform.translation.z
    robot_gps_pose.pose.pose.orientation = trans.transform.rotation
    robot_gps_pub.publish(robot_gps_pose)
    print "Transformation found"
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print "Can not find the transformation from " + robot_frame + " to utm_fiducial"
  
  rate.sleep()
  
