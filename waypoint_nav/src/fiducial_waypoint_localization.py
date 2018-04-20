#!/usr/bin/env python

import rospy
import math
import json
from pyproj import Proj

# TF libraries
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

# ROS messages
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialMapEntryArray, FiducialMapEntry, FiducialTransformArray

# Variables
global projection, tfBuffer, listener, gps_fiducials, reference_id, robot_stopped, fid_measurements
projection = Proj(proj="utm", zone="34", ellps='WGS84')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
reference_id = 0
robot_gps_pose = Odometry()
robot_stopped = False
fid_measurements = {} 

# Math functions
def degToRad(d):
  r = d * math.pi / 180.0
  return r

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
def mapGPSCB(GPS_map):
  return

def transCB(t):
  global reference_id, camera_frame, gps_frame, fid_ids, robot_gps_pose, robot_stopped, fid_measurements

  # Get id list
  for m in fid_measurements:
    flag = False
    for fid_trans in t.transforms:
      if str(fid_trans.fiducial_id) == m:
        flag = True
    if not flag:
      fid_measurements[m] = 0 
  
  # Process the messege 
  for fid_trans in t.transforms:   
     
    # save the id and name
    reference_id = fid_trans.fiducial_id
    fid_name = str(reference_id)  
    # skip if the tf is not in the map or image error is large
    if not fid_name in fid_measurements or fid_trans.image_error > 0.3:
      continue
    
    if fid_measurements[fid_name] == 0:
      # Pause the navigation and stop the robot 	
      state_msg = String()
      state_msg.data = "STOP"
      state_pub.publish(state_msg)
      # wait until robot stops
      while not robot_stopped:
        rate.sleep()
      # Wait another 5 seconds for updating the image
      i = 0
      while i < 5:
        rate.sleep()
        i = i + 1
        
      while 1:
        tf_fid_cam = geometry_msgs.msg.TransformStamped()
        tf_fid_cam.header.frame_id = camera_frame
        tf_fid_cam.child_frame_id = fid_name
        tf_fid_cam.header.stamp = rospy.Time.now()      
        tf_fid_cam.transform = fid_trans.transform
        tfmsg_fid_cam = tf2_msgs.msg.TFMessage([tf_fid_cam])
        tf_pub.publish(tfmsg_fid_cam)
        
        try: 
          robot_fid_trans = tfBuffer.lookup_transform(fid_name, robot_frame, rospy.Time())
          tf_robot_fid = geometry_msgs.msg.TransformStamped()
          tf_robot_fid.header.frame_id = "fiducial"+fid_name
          tf_robot_fid.child_frame_id = "robot_fid"
          tf_robot_fid.header.stamp = rospy.Time.now()      
          tf_robot_fid.transform = robot_fid_trans.transform
          tfmsg_robot_fid = tf2_msgs.msg.TFMessage([tf_robot_fid])
          tf_pub.publish(tfmsg_robot_fid)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          print "Can not find the transformation for robot_fid"        
       
        # Transform from robot to fiducial
        try:
          verify_trans = tfBuffer.lookup_transform(gps_frame, "fiducial"+fid_name, rospy.Time())
          verify_trans2 = tfBuffer.lookup_transform("fiducial"+fid_name, "robot_fid", rospy.Time())
          robot_utm_trans = tfBuffer.lookup_transform(gps_frame, "robot_fid", rospy.Time())
          robot_gps_pose.pose.pose.position.z = robot_utm_trans.transform.translation.z
          robot_gps_pose.pose.pose.position.x,robot_gps_pose.pose.pose.position.y = projection(robot_utm_trans.transform.translation.x, robot_utm_trans.transform.translation.y, inverse=True)
          robot_gps_pose.pose.pose.orientation.x = -robot_utm_trans.transform.rotation.x
          robot_gps_pose.pose.pose.orientation.y = -robot_utm_trans.transform.rotation.y
          robot_gps_pose.pose.pose.orientation.z = -robot_utm_trans.transform.rotation.z
          robot_gps_pose.pose.pose.orientation.w = robot_utm_trans.transform.rotation.w
          robot_gps_pose.pose.pose.orientation = quatRot(robot_gps_pose.pose.pose.orientation,0,0,90)
          robot_gps_pub.publish(robot_gps_pose)
          print "Transformation found"          
          # Resume the navigation when the update is done
          state_msg.data = "RUNNING"
          state_pub.publish(state_msg)          
          break
          fid_measurements[fid_name] = 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          print "Can not find the transformation"    
  
def serialCB(s):
  global robot_stopped
  if len(s.data) > 0: 
    line_parts = s.data.split('\t')
    try:
      v = float(line_parts[5])
      omega = float(line_parts[6])
      if math.fabs(v) < 0.05 and math.fabs(omega) < 0.05:
        robot_stopped = True
      else:
        robot_stopped = False
    except:
      return

# Init ROS node
rospy.init_node('fiducial_waypoint_localization')

# rosparams
global robot_frame, fiducial_frame, camera_frame, gps_frame
robot_frame = rospy.get_param("waypoint_control/base_frame", "base_footprint")
gps_frame = rospy.get_param("waypoint_control/gps_frame", "utm")
fiducial_map_file = rospy.get_param("waypoint_control/map_file", "/home/ros/catkin_ws/src/DTU-R3-ROS/waypoint_nav/src/Fiducials.json")
camera_frame = rospy.get_param("waypoint_control/camera_frame", "raspicam")
        
# Publishers
robot_gps_pub = rospy.Publisher('robot_gps_pose', Odometry, queue_size = 10, latch = True)
tf_pub = rospy.Publisher("tf", tf2_msgs.msg.TFMessage, queue_size=30, latch = True)
tf2_pub = rospy.Publisher("tf_static", tf2_msgs.msg.TFMessage, queue_size=30, latch = True)
state_pub = rospy.Publisher('waypoint/state', String, queue_size = 10)

# Subscribers
map_gps_sub = rospy.Subscriber('fiducial_map_gps', FiducialMapEntryArray, mapGPSCB)
detect_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, transCB)
serial_sub = rospy.Subscriber('serial', String, serialCB)

# 1 Hz
rate = rospy.Rate(1)

# Init fiducials map in GPS from json, publish all fiducial to utm trans to tf
json_data = json.load(open(fiducial_map_file))
fiducial_gps_map = FiducialMapEntryArray()
for fid in json_data["FiducialCollections"][0]["SavedFiducials"]:
  fid_measurements[str(fid["Id"])] = 0
  # axis y and z are reversed in Unity
  fid_utm_x, fid_utm_y = projection(fid["Position"]["longitude"], fid["Position"]["latitude"]) 
  quat = tf.transformations.quaternion_from_euler(-degToRad(fid["Rotation"]["east"]), -degToRad(fid["Rotation"]["north"]), -degToRad(fid["Rotation"]["heading"]))
  tf_fid_utm = geometry_msgs.msg.TransformStamped()
  tf_fid_utm.header.frame_id = gps_frame
  tf_fid_utm.child_frame_id = "fiducial" + str(fid["Id"])
  tf_fid_utm.header.stamp = rospy.Time.now()  
  tf_fid_utm.transform.translation.x = fid_utm_x
  tf_fid_utm.transform.translation.y = fid_utm_y
  tf_fid_utm.transform.translation.z = fid["Position"]["altitude"]
  tf_fid_utm.transform.rotation.x = quat[0]
  tf_fid_utm.transform.rotation.y = quat[1]
  tf_fid_utm.transform.rotation.z = quat[2]
  tf_fid_utm.transform.rotation.w = quat[3]
  tfmsg_fid_utm = tf2_msgs.msg.TFMessage([tf_fid_utm])
  tf2_pub.publish(tfmsg_fid_utm)
  rate.sleep()
  
while not rospy.is_shutdown():
  rate.sleep()
  
