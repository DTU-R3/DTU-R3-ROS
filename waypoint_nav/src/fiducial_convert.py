#!/usr/bin/env python

import rospy
import math

import tf
from sensor_msgs.msg import NavSatFix
from fiducial_msgs.msg import FiducialMapEntryArray, FiducialMapEntry
from geometry_msgs.msg import Point, Vector3, TransformStamped, PoseWithCovarianceStamped
from pyproj import Proj
from pyquaternion import Quaternion

# Callback functions 
def mapCB(fiducial_map):
  global tf_broadcaster, fiducial_frame
  for fid in fiducial_map.fiducials:
    frame_id = fiducial_frame
    child_frame_id = "fid"+str(fid.fiducial_id)
    quat = tf.transformations.quaternion_from_euler(fid.rx,fid.ry,fid.rz)
    tf_broadcaster.sendTransform( (fid.x, fid.y, fid.z), quat, rospy.Time.now(), child_frame_id, frame_id)
      
def mapGPSCB(GPS_map):
  global tf_broadcaster, utm_frame
  
  for fid in GPS_map.fiducials:
    fid_utm_x, fid_utm_y = projection(fid.x, fid.y)   
    frame_id = utm_frame
    child_frame_id = "fid"+str(fid.fiducial_id)   
    quat = tf.transformations.quaternion_from_euler(fid.rx,fid.ry,fid.rz)
    tf_broadcaster.sendTransform( (fid_utm_x, fid_utm_y, fid.z) , quat, rospy.Time.now(), child_frame_id, frame_id)

def poseCB(pose):
  global robot_frame, utm_frame
  try:
    (trans,rot) = listener.lookupTransform(robot_frame, utm_frame, rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print "Error: Can not find the transformation from robot to utm"
    
  gps_pose = NavSatFix()
  gps_pose.longitude,gps_pose.latitude = projection(trans.x, trans.y, inverse=True)
  gps_pose.altitude = trans.z
  robot_gps_pub.publish(gps_pose)
  gps_heading = Vector3()
  gps_heading = tf.transformations.euler_from_quaternion(rot)
  robot_heading.publish(gps_heading)
  
def pointCB(point):
  global fiducial_frame, utm_frame
  goal = Point()
  goal.x,goal.y = projection(point.longitude, point.latitude)
  goal.z = point.altitude
  try:
    (trans,rot) = listener.lookupTransform(utm_frame, fiducial_frame, rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print "Error: Can not find the transformation from robot to utm"
    
  goal.x = goal.x - trans.x
  goal.y = goal.y - trans.y
  goal.z = goal.z - trans.z
  q_rot = Quaternion(rot.w, rot.x, rot.y, rot.z)
  goal.x,goal.y,goal.z=q_rot.rotate([goal.x,goal.y,goal.z]) # waypoition position in fiducial map
  waypoint_pub.publish(goal)

# Init ROS node
rospy.init_node('fiducial_waypoint_convert')

# rosparams
global robot_frame, fiducial_frame
robot_frame = rospy.get_param("fiducial_slam/base_frame", "base_footprint")
fiducial_frame = rospy.get_param("fiducial_slam/map_frame", "map")
        
# Publishers
waypoint_pub = rospy.Publisher('waypoint/goal', Point, queue_size = 10)
robot_gps_pub = rospy.Publisher('robot_gps_pose', NavSatFix, queue_size = 10)
robot_heading = rospy.Publisher('robot_gps_heading', Vector3, queue_size = 10)

# Subscribers
pose_sub = rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, poseCB)
point_sub = rospy.Subscriber('waypoint', NavSatFix, pointCB)
map_sub = rospy.Subscriber('fiducial_map', FiducialMapEntryArray, mapCB)
map_gps_sub = rospy.Subscriber('fiducial_map_GPS', FiducialMapEntryArray, mapGPSCB)

rate = rospy.Rate(100)

global projection, utm_frame, reference_id, tf_broadcaster, listener
projection = Proj(proj="utm", zone="34", ellps='WGS84')
tf_broadcaster = tf.TransformBroadcaster()
reference_id = 129
utm_frame = "utm"
listener = tf.TransformListener()

while not rospy.is_shutdown():
  rate.sleep()
  
