#!/usr/bin/env python

import rospy
import json

from fiducial_msgs.msg import FiducialMapEntryArray, FiducialMapEntry

# Init ROS node
rospy.init_node('fiducial_map_test')

# Publishers
map_pub = rospy.Publisher('fiducial_map', FiducialMapEntryArray, queue_size = 10, latch = True)
map_gps_pub = rospy.Publisher('fiducial_map_GPS', FiducialMapEntryArray, queue_size = 10, latch = True)

data = json.load(open('/home/ros/catkin_ws/src/DTU-R3-ROS/waypoint_nav/src/Fiducials.json'))

rate = rospy.Rate(100)

index = len( data["FiducialCollections"][0]["SavedFiducials"])
fiducial_map = FiducialMapEntryArray()
fiducial_gps_map = FiducialMapEntryArray()

while not rospy.is_shutdown():
  for fid in data["FiducialCollections"][0]["SavedFiducials"]:

    fid_map = FiducialMapEntry()
    fid_map.fiducial_id = fid["Id"]
    fid_map.x = fid["OriginalData"]["X"]
    fid_map.y = fid["OriginalData"]["Y"]
    fid_map.z = fid["OriginalData"]["Z"]
    fid_map.rx = fid["OriginalData"]["RX"]
    fid_map.ry = fid["OriginalData"]["RY"]
    fid_map.rz = fid["OriginalData"]["RZ"]
    fiducial_map.fiducials.append(fid_map)
    
    fid_gps_map = FiducialMapEntry()
    fid_gps_map.fiducial_id = fid["Id"]
    fid_gps_map.x = fid["Position"]["longitude"]
    fid_gps_map.y = fid["Position"]["latitude"]
    fid_gps_map.z = fid["Position"]["altitude"]
    fid_gps_map.rx = fid["Rotation"]["x"]
    fid_gps_map.ry = fid["Rotation"]["y"]
    fid_gps_map.rz = fid["Rotation"]["z"]
    fiducial_gps_map.fiducials.append(fid_gps_map)

#  map_pub.publish(fiducial_map)
  map_gps_pub.publish(fiducial_gps_map)
  rate.sleep()
