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
global projection, tfBuffer, listener, gps_fiducials, reference_id, state, prestate, fiducial_gps_map, previous_fiducial
global l_counts, r_counts, l_displacement, r_displacement
projection = Proj(proj="utm", zone="34", ellps='WGS84')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
reference_id = 0
robot_gps_pose = Odometry()
fiducial_gps_map = FiducialMapEntryArray()
state = "STOP"
prestate = "STOP"
previous_fiducial = 0
l_counts = 0
r_counts = 0
l_displacement = 0
r_displacement = 0

global robot_state, STOP, RUNNING, WAIT_DONE, waiting_time, waiting
STOP = 0
RUNNING = 1
robot_state = STOP
waiting_time = 0
waiting = False

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
  global fiducial_gps_map
  fiducial_gps_map = GPS_map

def stateCB(s):
  global state
  state = s.data

def transCB(t):
  global reference_id, camera_frame, gps_frame, fid_ids, robot_gps_pose, state, prestate, waiting
  global robot_state, STOP, RUNNING, waiting_time, previous_fiducial, l_displacement, r_displacement, distance_per_count

  prev_in_view = False
  for fid_trans in t.transforms:  
    # Check whether the previous fiducial is out of the view
    if fid_trans.fiducial_id == previous_fiducial: 
      prev_in_view = True
      # if the displacement of both wheels is small
      if (l_displacement * float(distance_per_count) < 3.0) and (r_displacement * float(distance_per_count) < 3.0):
        return 
        
  if not prev_in_view:
    previous_fiducial = 0
  
  for fid_trans in t.transforms:
    # Check the image error
    if fid_trans.image_error > 0.3:
      return   
      
    # Check whether detected fiducial is in the map  
    for fid_gps in fiducial_gps_map.fiducials: 
      # Only process the detected fiducial in the map
      if fid_trans.fiducial_id != fid_gps.fiducial_id:
        continue
       
      reference_id = fid_trans.fiducial_id
      print reference_id
      fid_name = str(reference_id)
      
      # Pause the navigation and stop the robot
      if not waiting:
        prestate = state
      state_msg = String()
      state_msg.data = "STOP"
      state_pub.publish(state_msg)
      waiting = True
      
      # wait until robot stops
      if not robot_state == STOP:
        waiting_time = 0
        rate.sleep()
        return

      # Wait another 5 seconds for updating the image
      if waiting_time < 5:
        waiting_time = waiting_time + 1
        rate.sleep()
        return
        
      # Publish tf from fid to camera
      tf_fid_cam = geometry_msgs.msg.TransformStamped()
      tf_fid_cam.header.frame_id = camera_frame
      tf_fid_cam.child_frame_id = fid_name
      tf_fid_cam.header.stamp = rospy.Time.now()      
      tf_fid_cam.transform = fid_trans.transform
      tfmsg_fid_cam = tf2_msgs.msg.TFMessage([tf_fid_cam])
      tf_pub.publish(tfmsg_fid_cam)
      if robot_state == STOP:
        rate.sleep()

      # Publish tf from fid to utm
      tf_fid_utm = geometry_msgs.msg.TransformStamped()
      tf_fid_utm.header.frame_id = gps_frame
      tf_fid_utm.child_frame_id = "fiducial"+fid_name
      tf_fid_utm.header.stamp = rospy.Time.now() 
      tf_fid_utm.transform.translation.x, tf_fid_utm.transform.translation.y = projection(fid_gps.x, fid_gps.y)
      tf_fid_utm.transform.translation.z = fid_gps.z
      quat = tf.transformations.quaternion_from_euler(-degToRad(fid_gps.rx), -degToRad(fid_gps.ry), -degToRad(fid_gps.rz))
      tf_fid_utm.transform.rotation.x = quat[0] 
      tf_fid_utm.transform.rotation.y = quat[1] 
      tf_fid_utm.transform.rotation.z = quat[2] 
      tf_fid_utm.transform.rotation.w = quat[3] 
      tfmsg_fid_utm = tf2_msgs.msg.TFMessage([tf_fid_utm])
      tf_pub.publish(tfmsg_fid_utm)
      if not robot_state == STOP:
        rate.sleep()

      # Publish tf from robot to fid
      try: 
        robot_fid_trans = tfBuffer.lookup_transform(fid_name, robot_frame, rospy.Time())
        tf_robot_fid = geometry_msgs.msg.TransformStamped()
        tf_robot_fid.header.frame_id = "fiducial"+fid_name
        tf_robot_fid.child_frame_id = "robot_fid"
        tf_robot_fid.header.stamp = rospy.Time.now()      
        tf_robot_fid.transform = robot_fid_trans.transform
        tfmsg_robot_fid = tf2_msgs.msg.TFMessage([tf_robot_fid])
        tf_pub.publish(tfmsg_robot_fid)        
        if robot_state == STOP:
          rate.sleep()
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return
      
      # Publish tf from robot to utm
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
        # Resume the navigation when the update is done
        state_msg.data = prestate
        state_pub.publish(state_msg)         
        waiting = False 
        previous_fiducial = reference_id
        l_displacement = 0
        r_displacement = 0
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):    
        return

def serialCB(s):
  global robot_state, STOP, RUNNING, l_counts, r_counts, l_displacement, r_displacement
  if len(s.data) > 0: 
    line_parts = s.data.split('\t')
    try:
      v = float(line_parts[5])
      omega = float(line_parts[6])
      new_l_counts = int(line_parts[7])
      new_r_counts = int(line_parts[8])
      # Check whether the robot stops
      if math.fabs(v) < 0.05 and math.fabs(omega) < 0.05:
        robot_state = STOP
      else:
        robot_state = RUNNING   
      l_displacement = l_displacement + math.fabs(new_l_counts - l_counts)
      r_displacement = r_displacement + math.fabs(new_r_counts - r_counts)
      l_counts = new_l_counts
      r_counts = new_r_counts
    except:
     return

# Init ROS node
rospy.init_node('fiducial_waypoint_localization')

# rosparams
global robot_frame, fiducial_frame, camera_frame, gps_frame, distance_per_count
robot_frame = rospy.get_param("~waypoint_control/base_frame", "base_footprint")
gps_frame = rospy.get_param("~waypoint_control/gps_frame", "utm")
fiducial_map_file = rospy.get_param("~waypoint_control/map_file", "/home/pi/catkin_ws/src/DTU-R3-ROS/waypoint_nav/src/Fiducials.json")
camera_frame = rospy.get_param("~waypoint_control/camera_frame", "raspicam")
distance_per_count = rospy.get_param("~driveGeometry/distancePerCount", "0.00338")
        
# Publishers
robot_gps_pub = rospy.Publisher('robot_gps_pose', Odometry, queue_size = 10, latch = True)
tf_pub = rospy.Publisher("tf", tf2_msgs.msg.TFMessage, queue_size=30, latch = True)
state_pub = rospy.Publisher('waypoint/state', String, queue_size = 10, latch = True)

# Subscribers
map_gps_sub = rospy.Subscriber('fiducial_map_gps', FiducialMapEntryArray, mapGPSCB)
detect_sub = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, transCB)
serial_sub = rospy.Subscriber('serial', String, serialCB)
state_sub = rospy.Subscriber('waypoint/state', String, stateCB)

# 1 Hz
rate = rospy.Rate(1)

# Init fiducials map in GPS from json, publish all fiducial to utm trans to tf
json_data = json.load(open(fiducial_map_file))
# Save the map in FiducialMapEntryArray()
for fid in json_data["FiducialCollections"][0]["SavedFiducials"]:
  fid_gps = FiducialMapEntry()
  fid_gps.fiducial_id = fid["Id"]
  fid_gps.x = fid["Position"]["longitude"]
  fid_gps.y = fid["Position"]["latitude"]
  fid_gps.z = fid["Position"]["altitude"]
  fid_gps.rx = fid["Rotation"]["east"]
  fid_gps.ry = fid["Rotation"]["north"]
  fid_gps.rz = fid["Rotation"]["heading"]
  fiducial_gps_map.fiducials.append(fid_gps)
  
while not rospy.is_shutdown():
  rate.sleep()
  
