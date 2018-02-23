#!/usr/bin/env python

import rospy
import math
from pyproj import Proj
from pyquaternion import Quaternion

# TF libraries
import tf
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Pose

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
def odomCB(odo):
  global robot_odom
  robot_odom = odo

def poseCB(p):
  global robot_pose, robot_odom
  robot_pose = p.pose.pose
  robot_pose.position.x, robot_pose.position.y = projection(p.pose.pose.position.x, p.pose.pose.position.y)
  
  # odom to utm
  try:
    odo_ref_trans = tfBuffer.lookup_transform(robot_frame, odom_frame, rospy.Time())
    tf_odo_ref = geometry_msgs.msg.TransformStamped()
    tf_odo_ref.header.frame_id = "odom_utm_refer"
    tf_odo_ref.child_frame_id = odom_frame
    tf_odo_ref.header.stamp = rospy.Time.now()      
    tf_odo_ref.transform = odo_ref_trans.transform
    tfmsg_odo_ref = tf2_msgs.msg.TFMessage([tf_odo_ref])
    tf_pub.publish(tfmsg_odo_ref)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print "Can not do the transformation from " + odom_frame + " to reference" 
  
  # reference to odom
  tf_r_o = geometry_msgs.msg.TransformStamped()
  tf_r_o.header.frame_id = odom_frame
  tf_r_o.header.stamp = rospy.Time.now()   
  tf_r_o.child_frame_id = "odom_utm_bridge"
  tf_r_o.transform.translation.x = robot_odom.pose.pose.position.x
  tf_r_o.transform.translation.y = robot_odom.pose.pose.position.y
  tf_r_o.transform.translation.z = robot_odom.pose.pose.position.z
  tf_r_o.transform.rotation = robot_odom.pose.pose.orientation
  tfmsg_r_o = tf2_msgs.msg.TFMessage([tf_r_o])
  tf_pub.publish(tfmsg_r_o)
  
  # utm to reference
  trans_reference_utm = geometry_msgs.msg.Transform()
  trans_reference_utm.translation.x = robot_pose.position.x
  trans_reference_utm.translation.y = robot_pose.position.y
  trans_reference_utm.translation.z = robot_pose.position.z
  trans_reference_utm.rotation = robot_pose.orientation
  inverseTrans("odom_utm_bridge", gps_frame, trans_reference_utm)
  
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
tf_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)

# Subscribers
robot_gps_sub = rospy.Subscriber('robot_gps_pose', Odometry, poseCB)
odom_sub = rospy.Subscriber('odom', Odometry, odomCB)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
  try:
    trans = tfBuffer.lookup_transform(gps_frame, odom_frame, rospy.Time())
    robot_odom_pose = PoseStamped()
    robot_odom_pose.pose = robot_odom.pose.pose
    pose_transformed = tf2_geometry_msgs.do_transform_pose(robot_odom_pose, trans)
    robot_gps_pose.pose.pose = pose_transformed.pose
    robot_gps_pose.pose.pose.position.x,robot_gps_pose.pose.pose.position.y = projection(pose_transformed.pose.position.x,pose_transformed.pose.position.y,inverse=True)
    robot_gps_pub.publish(robot_gps_pose)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    continue
  rate.sleep()
  
