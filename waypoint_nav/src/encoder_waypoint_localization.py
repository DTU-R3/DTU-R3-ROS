#!/usr/bin/env python

import rospy
import math
from pyproj import Proj
from R3_functions import quat_rot, fit_in_rad, debug_info
from bisect import bisect_left

# TF libraries
import tf
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs

# ROS messages
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

# Class
class encoder_localization(object):
  def __init__(self):
    # Variables
    self.projection = Proj(proj="utm", zone="34", ellps='WGS84')
    self.tfBuffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
    self.robot_odom = Odometry()
    self.odom_list = [[],[]] # a list contains timestamp list and odom_pose list
    self.odom_calibrating = False
    self.list_cleaning = False
    
    # Init ROS node
    rospy.init_node('encoder_waypoint_localization')
    
    # rosparams
    self.robot_frame = rospy.get_param("~waypoint_control/base_frame", "base_footprint")
    self.gps_frame = rospy.get_param("~waypoint_control/gps_frame", "utm")
    self.odom_frame = rospy.get_param("~waypoint_control/odom_frame", "odom")

    # Publishers
    self.robot_gps_pub = rospy.Publisher('robot_gps_pose', Odometry, queue_size = 10)
    self.tf_pub = rospy.Publisher("tf", tf2_msgs.msg.TFMessage, queue_size=20, latch = True)
    self.tf2_pub = rospy.Publisher("tf_static", tf2_msgs.msg.TFMessage, queue_size=20, latch = True)
    self.debug_output = rospy.Publisher('debug_output', String, queue_size = 10)
    
    # Subscribers
    rospy.Subscriber('odo_calib_pose', Odometry, self.poseCB)
    rospy.Subscriber('odom', Odometry, self.odomCB)

    self.freq = 2
    self.rate = rospy.Rate(self.freq)

  def Start(self):
    while not rospy.is_shutdown():
      self.rate.sleep()

  # ROS Callback functions
  def poseCB(self, p):
    self.odom_calibrating = True
    robot_pose = Pose()
    robot_pose = p.pose.pose
    robot_pose.position.x, robot_pose.position.y = self.projection(p.pose.pose.position.x, p.pose.pose.position.y)
    robot_pose.orientation.x = -p.pose.pose.orientation.x
    robot_pose.orientation.y = -p.pose.pose.orientation.y
    robot_pose.orientation.z = -p.pose.pose.orientation.z
    robot_pose.orientation = quat_rot(robot_pose.orientation,0,0,90)
    debug_info(self.debug_output, "Odom calib tf received")
    
    # Align odometry with odom_calib and calculate offset
    current_stamp = p.header.stamp
    i = bisect_left(self.odom_list[0], current_stamp)
    if i == len(self.odom_list[0]):
      i -= 1
    elif self.odom_list[0][i] - current_stamp > current_stamp - self.odom_list[0][i]:
      i -= 1
    current_euler = tf.transformations.euler_from_quaternion((self.robot_odom.pose.pose.orientation.x, self.robot_odom.pose.pose.orientation.y, self.robot_odom.pose.pose.orientation.z, self.robot_odom.pose.pose.orientation.w))
    bench_euler = tf.transformations.euler_from_quaternion((self.odom_list[1][i].orientation.x, self.odom_list[1][i].orientation.y, self.odom_list[1][i].orientation.z, self.odom_list[1][i].orientation.w))
    offset_odom_x = self.robot_odom.pose.pose.position.x - self.odom_list[1][i].position.x
    offset_odom_y = self.robot_odom.pose.pose.position.y - self.odom_list[1][i].position.y
    offset_odom_z = self.robot_odom.pose.pose.position.z - self.odom_list[1][i].position.z
    offset_odom_rz = current_euler[2] - bench_euler[2]
    # Remove out dated elements after calculating offset
    self.list_cleaning = True
    self.odom_list[0] = self.odom_list[0][i+1:]
    self.odom_list[1] = self.odom_list[1][i+1:]
    self.list_cleaning = False
    while True:
      try:
        odo_utm_trans = self.tfBuffer.lookup_transform(self.gps_frame, self.odom_frame, rospy.Time())
        odo_utm_euler = tf.transformations.euler_from_quaternion((odo_utm_trans.transform.rotation.x, odo_utm_trans.transform.rotation.y, odo_utm_trans.transform.rotation.z, odo_utm_trans.transform.rotation.w))
        theta = odo_utm_euler[2]
        break
      except:
        pass
    robot_pose.position.x += offset_odom_x * math.cos(theta) - offset_odom_y * math.sin(theta)
    robot_pose.position.y += offset_odom_x * math.sin(theta) + offset_odom_y * math.cos(theta)
    robot_pose.position.z += offset_odom_z
    robot_pose.orientation = quat_rot(robot_pose.orientation,0,0,math.degrees(offset_odom_rz))
    debug_info(self.debug_output, "Odometry offset calculated")
    
    # odom to reference
    while True:
      try:
        odo_ref_trans = self.tfBuffer.lookup_transform(self.robot_frame, self.odom_frame, rospy.Time())
        tf_odo_ref = TransformStamped()
        tf_odo_ref.header.frame_id = "odom_utm_calib"
        tf_odo_ref.child_frame_id = self.odom_frame
        tf_odo_ref.header.stamp = rospy.Time.now()      
        tf_odo_ref.transform = odo_ref_trans.transform
        tfmsg_odo_ref = tf2_msgs.msg.TFMessage([tf_odo_ref])
        self.tf_pub.publish(tfmsg_odo_ref)
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        debug_info(self.debug_output, "Publishing odometry calibration")
        continue
  
      # reference to utm
      tf_ref_utm = TransformStamped()
      tf_ref_utm.header.frame_id = self.gps_frame
      tf_ref_utm.child_frame_id = "odom_utm_calib"
      tf_ref_utm.header.stamp = rospy.Time.now()  
      tf_ref_utm.transform.translation.x = robot_pose.position.x
      tf_ref_utm.transform.translation.y = robot_pose.position.y
      tf_ref_utm.transform.translation.z = robot_pose.position.z
      tf_ref_utm.transform.rotation = robot_pose.orientation
      tfmsg_ref_utm = tf2_msgs.msg.TFMessage([tf_ref_utm])
      self.tf2_pub.publish(tfmsg_ref_utm)
      self.rate.sleep()
    
      # Check the tf exists correctly
      try:
        trans = self.tfBuffer.lookup_transform(self.gps_frame, "odom_utm_calib", rospy.Time())
        trans2 = self.tfBuffer.lookup_transform("odom_utm_calib", self.odom_frame, rospy.Time())
        if math.fabs(trans.transform.translation.x - robot_pose.position.x) > 0.1 or math.fabs(trans.transform.translation.y - robot_pose.position.y) > 0.1:
          continue
        self.odom_calibrating = False
        debug_info(self.debug_output, "Odometry calibrated")
        break
      except:
        continue  
      
  def odomCB(self, odo):
    self.robot_odom = odo
    if not self.list_cleaning:
      self.odom_list[0].append(odo.header.stamp)
      self.odom_list[1].append(odo.pose.pose)
    
    if self.odom_calibrating:
        self.rate.sleep()
        return
    try:
      trans = self.tfBuffer.lookup_transform(self.gps_frame, self.odom_frame, rospy.Time())
      robot_odom_pose = PoseStamped()
      robot_odom_pose.pose = self.robot_odom.pose.pose
      pose_transformed = tf2_geometry_msgs.do_transform_pose(robot_odom_pose, trans)
      robot_gps_pose = Odometry()
      robot_gps_pose.header.frame_id = self.gps_frame
      robot_gps_pose.child_frame_id = self.robot_frame
      robot_gps_pose.pose.pose = pose_transformed.pose
      robot_gps_pose.pose.pose.position.x,robot_gps_pose.pose.pose.position.y = self.projection(pose_transformed.pose.position.x,pose_transformed.pose.position.y,inverse=True)
      robot_gps_pose.pose.pose.orientation.x = -pose_transformed.pose.orientation.x
      robot_gps_pose.pose.pose.orientation.y = -pose_transformed.pose.orientation.y
      robot_gps_pose.pose.pose.orientation.z = -pose_transformed.pose.orientation.z
      robot_gps_pose.pose.pose.orientation = quat_rot(robot_gps_pose.pose.pose.orientation,0,0,90)
      self.robot_gps_pub.publish(robot_gps_pose)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      return
    
if __name__ == '__main__': 
  encoder = encoder_localization() 
  encoder.Start()
  
