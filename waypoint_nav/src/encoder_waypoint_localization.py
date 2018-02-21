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

# ROS messages
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

# Variables
global projection, tfBuffer, listener, robot_odom, robot_pose, robot_gps_pose
projection = Proj(proj="utm", zone="34", ellps='WGS84')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
robot_odom = Odometry()
robot_pose = Pose()
robot_gps_pose = Odometry()

# ROS Callback functions
def odomCB(odo):
  global robot_odom
  robot_odom = odo

def poseCB(p):
  global robot_pose, robot_odom
  robot_pose = p.pose.pose
  robot_pose.position.x, robot_pose.position.y = projection(p.pose.pose.position.x, p.pose.pose.position.y)
  
  # reference to odom
  tf_r_o = geometry_msgs.msg.TransformStamped()
  tf_r_o.header.frame_id = odom_frame
  tf_r_o.header.stamp = rospy.Time.now()   
  tf_r_o.child_frame_id = "reference"
  tf_r_o.transform.translation.x = robot_odom.pose.pose.position.x
  tf_r_o.transform.translation.y = robot_odom.pose.pose.position.y
  tf_r_o.transform.translation.z = robot_odom.pose.pose.position.z
  tf_r_o.transform.rotation = robot_odom.pose.pose.orientation
  tfmsg_r_o = tf2_msgs.msg.TFMessage([tf_r_o])
  tf_pub.publish(tfmsg_r_o)
  
  # reference to utm
  tf_r_u = geometry_msgs.msg.TransformStamped()
  tf_r_u.header.frame_id = "utm_test"
  tf_r_u.header.stamp = rospy.Time.now()   
  tf_r_u.child_frame_id = "reference_test"
  tf_r_u.transform.translation.x = robot_pose.position.x
  tf_r_u.transform.translation.y = robot_pose.position.y
  tf_r_u.transform.translation.z = robot_pose.position.z
  tf_r_u.transform.rotation = robot_pose.orientation
  tfmsg_r_u = tf2_msgs.msg.TFMessage([tf_r_u])
  tf_pub.publish(tfmsg_r_u)
  
  # utm to reference / inverse reference_test to utm_test
  try:
    trans = tfBuffer.lookup_transform("reference_test", "utm_test", rospy.Time())
    tf_u_r = geometry_msgs.msg.TransformStamped()
    tf_u_r.header.frame_id = "reference"
    tf_u_r.header.stamp = rospy.Time.now()   
    tf_u_r.child_frame_id = utm_frame
    tf_u_r.transform = trans.transform
    tfmsg_u_r = tf2_msgs.msg.TFMessage([tf_u_r])
    tf_pub.publish(tfmsg_u_r) 
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print "Can not find the transformation from " + utm_frame + " to reference"
  
# Init ROS node
rospy.init_node('encoder_waypoint_localization')

# rosparams
global robot_frame, gps_frame, odom_frame
robot_frame = rospy.get_param("waypoint_control/base_frame", "base_footprint")
gps_frame = rospy.get_param("waypoint_control/gps_frame", "utm")
odom_frame = rospy.get_param("waypoint_control/odom_frame", "odom")
        
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
    print "Can not find the transformation from " + odom_frame + " to " + gps_frame
  rate.sleep()
  
