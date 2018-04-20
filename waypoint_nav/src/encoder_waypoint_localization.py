#!/usr/bin/env python

import rospy
import math
from pyproj import Proj

# TF libraries
import tf
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped

# ROS messages
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


# Variables
global projection, tfBuffer, listener, robot_odom, robot_pose, robot_gps_pose
projection = Proj(proj="utm", zone="34", ellps='WGS84')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
robot_odom = Odometry()
robot_pose = Pose()
robot_gps_pose = Odometry()

def quatRot(q,deg_x,deg_y,deg_z):
  euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
  quat = tf.transformations.quaternion_from_euler(euler[0]+deg_x*math.pi/180, euler[1]+deg_y*math.pi/180, euler[2]+deg_z*math.pi/180)
  result = geometry_msgs.msg.Quaternion()
  result.x = quat[0]
  result.y = quat[1]
  result.z = quat[2]
  result.w = quat[3]
  return result
  
# ROS Callback functions
def odomCB(odo):
  global robot_odom
  robot_odom = odo

def poseCB(p):
  global robot_pose, robot_odom, odom_frame, gps_frame
  robot_pose = p.pose.pose
  robot_pose.position.x, robot_pose.position.y = projection(p.pose.pose.position.x, p.pose.pose.position.y)
  robot_pose.orientation.x = -p.pose.pose.orientation.x
  robot_pose.orientation.y = -p.pose.pose.orientation.y
  robot_pose.orientation.z = -p.pose.pose.orientation.z
  robot_pose.orientation = quatRot(robot_pose.orientation,0,0,90)
  
  while 1:
    # odom to reference
    try:
      odo_ref_trans = tfBuffer.lookup_transform(robot_frame, odom_frame, rospy.Time())
      tf_odo_ref = geometry_msgs.msg.TransformStamped()
      tf_odo_ref.header.frame_id = "odom_utm_calib"
      tf_odo_ref.child_frame_id = odom_frame
      tf_odo_ref.header.stamp = rospy.Time.now()      
      tf_odo_ref.transform = odo_ref_trans.transform
      tfmsg_odo_ref = tf2_msgs.msg.TFMessage([tf_odo_ref])
      tf_pub.publish(tfmsg_odo_ref)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print "Can not do the transformation from " + odom_frame + " to reference" 
  
    # reference to utm
    tf_ref_utm = geometry_msgs.msg.TransformStamped()
    tf_ref_utm.header.frame_id = gps_frame
    tf_ref_utm.header.stamp = rospy.Time.now()   
    tf_ref_utm.child_frame_id = "odom_utm_calib"
    tf_ref_utm.transform.translation.x = robot_pose.position.x
    tf_ref_utm.transform.translation.y = robot_pose.position.y
    tf_ref_utm.transform.translation.z = robot_pose.position.z
    tf_ref_utm.transform.rotation = robot_pose.orientation
    tfmsg_ref_utm = tf2_msgs.msg.TFMessage([tf_ref_utm])
    tf2_pub.publish(tfmsg_ref_utm)
    rate.sleep()
    
    # Check the tf exists correctly
    try:
      trans = tfBuffer.lookup_transform(gps_frame, "odom_utm_calib", rospy.Time())
      trans2 = tfBuffer.lookup_transform("odom_utm_calib", odom_frame, rospy.Time())
      break
    except:
      print "The transformation has not been created yet"    

# Init ROS node
rospy.init_node('encoder_waypoint_localization')

# rosparams
global robot_frame, gps_frame, odom_frame
robot_frame = rospy.get_param("waypoint_control/base_frame", "base_footprint")
gps_frame = rospy.get_param("waypoint_control/gps_frame", "utm")
odom_frame = rospy.get_param("waypoint_control/odom_frame", "odom")

robot_gps_pose.header.frame_id = gps_frame
robot_gps_pose.child_frame_id = robot_frame
    
# Publishers
robot_gps_pub = rospy.Publisher('robot_gps_pose', Odometry, queue_size = 10)
tf_pub = rospy.Publisher("tf", tf2_msgs.msg.TFMessage, queue_size=20, latch = True)
tf2_pub = rospy.Publisher("tf_static", tf2_msgs.msg.TFMessage, queue_size=20, latch = True)

# Subscribers
robot_gps_sub = rospy.Subscriber('odo_calib_pose', Odometry, poseCB)
odom_sub = rospy.Subscriber('odom', Odometry, odomCB)

freq = 10
rate = rospy.Rate(freq)

while not rospy.is_shutdown():
  try:
    trans = tfBuffer.lookup_transform(gps_frame, odom_frame, rospy.Time())
    robot_odom_pose = PoseStamped()
    robot_odom_pose.pose = robot_odom.pose.pose
    pose_transformed = tf2_geometry_msgs.do_transform_pose(robot_odom_pose, trans)
    robot_gps_pose.pose.pose = pose_transformed.pose
    robot_gps_pose.pose.pose.position.x,robot_gps_pose.pose.pose.position.y = projection(pose_transformed.pose.position.x,pose_transformed.pose.position.y,inverse=True)
    robot_gps_pose.pose.pose.orientation.x = -pose_transformed.pose.orientation.x
    robot_gps_pose.pose.pose.orientation.y = -pose_transformed.pose.orientation.y
    robot_gps_pose.pose.pose.orientation.z = -pose_transformed.pose.orientation.z
    robot_gps_pose.pose.pose.orientation = quatRot(robot_gps_pose.pose.pose.orientation,0,0,90)
    robot_gps_pub.publish(robot_gps_pose)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    continue
  rate.sleep()
  
