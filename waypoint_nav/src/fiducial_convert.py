#!/usr/bin/env python

import rospy
import math

import tf
from sensor_msgs.msg import NavSatFix
from fiducial_msgs.msg import FiducialMapEntryArray
from geometry_msgs.msg import Point, Vector3, TransformStamped, PoseWithCovarianceStamped
from pyproj import Proj
from pyquaternion import Quaternion

# Callback functions 
def mapCB(fiducial_map):
  global reference_id, tf_broadcaster, reference_fiducial_tf
  if reference_id==0:
    reference_id = fiducial_map.fiducials[0].fiducial_id
  for fid in fiducial_map.fiducials:
    if fid.fiducial_id == reference_id:
      reference_fiducial_tf.transform.translation.x = fid.x
      reference_fiducial_tf.transform.translation.y = fid.y
      reference_fiducial_tf.transform.translation.z = fid.z
      quat = tf.transformations.quaternion_from_euler(fid.rx,fid.ry,fid.rz)
      reference_fiducial_tf.transform.rotation.x = quat[0]
      reference_fiducial_tf.transform.rotation.y = quat[1]
      reference_fiducial_tf.transform.rotation.z = quat[2]
      reference_fiducial_tf.transform.rotation.w = quat[3]
      tf_broadcaster.sendTransform(reference_fiducial_tf)
      
def mapGPSCB(GPS_map):
  global reference_id, tf_broadcaster, reference_utm_tf, utm_map
  if reference_id==0:
    reference_id = GPS_map.fiducials[0].fiducial_id
  
  i = 0  
  for fid in GPS_map.fiducials:
    utm_map.fiducials[i]=fid
    utm_map.fiducials[i].x,utm_map.fiducials[i].y = projection(fid.x, fid.y)
    i=i+1      
    
  for fid in utm_map.fiducials:
    if fid.fiducial_id == reference_id:
      reference_utm_tf.transform.translation.x = fid.x
      reference_utm_tf.transform.translation.y = fid.y
      reference_utm_tf.transform.translation.z = fid.z
      quat = tf.transformations.quaternion_from_euler(fid.rx,fid.ry,fid.rz)
      reference_utm_tf.transform.rotation.x = quat[0]
      reference_utm_tf.transform.rotation.y = quat[1]
      reference_utm_tf.transform.rotation.z = quat[2]
      reference_utm_tf.transform.rotation.w = quat[3]
      tf_broadcaster.sendTransform(reference_utm_tf)

def poseCB(pose):
  global fiducial_utm_tf
  robot_fiducial_tf = TransformStamped()
  robot_fiducial_tf.header.frame_id = "fiducial"
  robot_fiducial_tf.child_frame_id = "robot"
  robot_fiducial_tf.transform.translation.x = pose.pose.pose.position.x
  robot_fiducial_tf.transform.translation.y = pose.pose.pose.position.y
  robot_fiducial_tf.transform.translation.z = pose.pose.pose.position.z
  robot_fiducial_tf.transform.rotation = pose.pose.pose.orientation
  robot_utm_tf = tfBuffer.lookup_transform('robot', 'utm', rospy.Time())
  gps_pose = NavSatFix()
  gps_pose.longitude,gps_pose.latitude = projection(robot_utm_tf.transform.translation.x,robot_utm_tf.transform.translation.y,inverse=True)
  gps_pose.altitude = robot_utm_tf.transform.translation.z
  robot_gps_pub.publish(gps_pose)
  gps_heading = Vector3()
  gps_heading = tf.transformations.euler_from_quaternion(robot_gps_tf.transform.orentation)
  robot_heading.publish(gps_heading)
  
def pointCB(point):
  global utm_fiducial_tf
  goal = Point()
  goal.x,goal.y = projection(point.longitude, point.latitude)
  goal.z = point.altitude
  goal.x = goal.x -utm_fiducial_tf.transform.translation.x
  goal.y = goal.y -utm_fiducial_tf.transform.translation.y
  goal.z = goal.z -utm_fiducial_tf.transform.translation.z
  q_rot = Quaternion(utm_fiducial_tf.transform.rotation.w, utm_fiducial_tf.transform.rotation.x, utm_fiducial_tf.transform.rotation.y, utm_fiducial_tf.transform.rotation.z)
  goal.x,goal.y,goal.z=q_rot.rotate([goal.x,goal.y,goal.z]) # waypoition position in fiducial
  waypoint_pub.publish(goal)

# Init ROS node
rospy.init_node('fiducial_waypoint_convert')

# Publishers
waypoint_pub = rospy.Publisher('waypoint/goal', Point, queue_size = 10)
robot_gps_pub = rospy.Publisher('robot_gps_pose', NavSatFix, queue_size = 10)
robot_heading = rospy.Publisher('robot_gps_heading', Vector3, queue_size = 10)

# Subscribers
pose_sub = rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, poseCB)
point_sub = rospy.Subscriber('waypoint', NavSatFix, pointCB)
map_sub = rospy.Subscriber('fiducial_map', FiducialMapEntryArray, mapCB)
map_gps_sub = rospy.Subscriber('fiducial_map_GPS', FiducialMapEntryArray, mapGPSCB)

rate = rospy.Rate(10)

global projection, utm_map, reference_id
global tf_broadcaster, reference_fiducial_tf, reference_utm_tf, utm_fiducial_tf, fiducial_utm_tf
projection = Proj(proj="utm", zone="34", ellps='WGS84')
tf_broadcaster = tf.TransformBroadcaster()
utm_map = FiducialMapEntryArray()
reference_id = 0	# should be auto update in the future
reference_fiducial_tf = TransformStamped()
reference_fiducial_tf.header.frame_id = "fiducial"
reference_fiducial_tf.child_frame_id = "reference_fid"
reference_utm_tf = TransformStamped()
reference_utm_tf.header.frame_id = "utm"
reference_utm_tf.child_frame_id = "reference_fid"
listener = tf.TransformListener()

while not rospy.is_shutdown():
  try:
    utm_fiducial_tf = tfBuffer.lookup_transform('utm', 'fiducial', rospy.Time())
    fiducial_utm_tf = tfBuffer.lookup_transform('fiducial', 'utm', rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print "Error when looking up the transformation"
    continue
  rate.sleep()
  
