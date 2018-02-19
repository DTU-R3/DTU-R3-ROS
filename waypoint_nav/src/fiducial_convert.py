#!/usr/bin/env python

import rospy
import math

import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import NavSatFix
from fiducial_msgs.msg import FiducialMapEntryArray, FiducialMapEntry
from geometry_msgs.msg import Point, Vector3, TransformStamped, PoseWithCovarianceStamped
from pyproj import Proj
from pyquaternion import Quaternion


#def mapCB(fiducial_map):
#  global fiducial_frame
#  for fid in fiducial_map.fiducials:
#    t = geometry_msgs.msg.TransformStamped()
#    t.header.frame_id = fiducial_frame
#    t.header.stamp = rospy.Time.now()   
#    t.child_frame_id = "fid"+str(fid.fiducial_id)
#    quat = tf.transformations.quaternion_from_euler(fid.rx,fid.ry,fid.rz)
#    t.transform.translation.x = fid.x
#    t.transform.translation.y = fid.y
#    t.transform.translation.z = fid.z
#    t.transform.rotation = quat
#    tfm = tf2_msgs.msg.TFMessage([t])
#    tf_pub.publish(tfm)

      
def mapGPSCB(GPS_map):
  global utm_frame, gps_fiducials
  gps_fiducials = GPS_map
#  for fid in GPS_map.fiducials:
#    t = geometry_msgs.msg.TransformStamped()
#    fid_utm_x, fid_utm_y = projection(fid.x, fid.y)   
#    t.header.frame_id = "fid"+str(fid.fiducial_id)
#    t.header.stamp = rospy.Time.now()   
#    t.child_frame_id = utm_frame
#    quat = tf.transformations.quaternion_from_euler(fid.rx,fid.ry,fid.rz)
#    t.transform.translation.x = fid_utm_x
#    t.transform.translation.y = fid_utm_y
#    t.transform.translation.z = fid.z
#    t.transform.rotation.x = quat[0]
#    t.transform.rotation.y = quat[1]
#    t.transform.rotation.z = quat[2]
#    t.transform.rotation.w = quat[3]
#    tfm = tf2_msgs.msg.TFMessage([t])
#    tf_pub.publish(tfm)

def poseCB(pose):
  global robot_frame, utm_frame, tfBuffer, listener
  trans = tfBuffer.lookup_transform(robot_frame, utm_frame, rospy.Time())
  gps_pose = NavSatFix()
  gps_pose.longitude,gps_pose.latitude = projection(trans.transform.translation.x, trans.transform.translation.y, inverse=True)
  gps_pose.altitude = trans.transform.translation.z
  robot_gps_pub.publish(gps_pose)
  gps_heading = Vector3()
  euler = tf.transformations.euler_from_quaternion((trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
  gps_heading.x = euler[0]
  gps_heading.y = euler[1]
  gps_heading.z = euler[2]
  robot_heading_pub.publish(gps_heading)
  
def pointCB(point):
  global fiducial_frame, utm_frame, tfBuffer, listener
  goal = Point()
  goal.x,goal.y = projection(point.longitude, point.lrostopatitude)
  goal.z = point.altitude
  (trans,rot) = listener.lookup_transform(utm_frame, fiducial_frame, rospy.Time())
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
robot_heading_pub = rospy.Publisher('robot_gps_heading', Vector3, queue_size = 10)
tf_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)

# Subscribers
pose_sub = rospy.Subscriber('fiducial_pose', PoseWithCovarianceStamped, poseCB)
point_sub = rospy.Subscriber('waypoint', NavSatFix, pointCB)
# map_sub = rospy.Subscriber('fiducial_map', FiducialMapEntryArray, mapCB)
map_gps_sub = rospy.Subscriber('fiducial_map_GPS', FiducialMapEntryArray, mapGPSCB)

rate = rospy.Rate(100)

global projection, utm_frame, tfBuffer, listener, gps_fiducials, reference_id
projection = Proj(proj="utm", zone="34", ellps='WGS84')
utm_frame = "utm"
reference_id = 129
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
gps_fiducials = FiducialMapEntryArray();

while not rospy.is_shutdown():
  for fid in gps_fiducials.fiducials:
    if fid.fiducial_id == reference_id:
      t = geometry_msgs.msg.TransformStamped()
      fid_utm_x, fid_utm_y = projection(fid.x, fid.y)   
      t.header.frame_id = "fid"+str(fid.fiducial_id)
      t.header.stamp = rospy.Time.now()   
      t.child_frame_id = utm_frame
      quat = tf.transformations.quaternion_from_euler(fid.rx,fid.ry,fid.rz)
      t.transform.translation.x = fid_utm_x
      t.transform.translation.y = fid_utm_y
      t.transform.translation.z = fid.z
      t.transform.rotation.x = quat[0]
      t.transform.rotation.y = quat[1]
      t.transform.rotation.z = quat[2]
      t.transform.rotation.w = quat[3]
      # inverse transformation
      t.transform = btTransform.inverse(t.transform)
      tfm = tf2_msgs.msg.TFMessage([t])
      tf_pub.publish(tfm)
  rate.sleep()
  
