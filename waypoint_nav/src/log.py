#!/usr/bin/env python

import rospy
import csv
import datetime

from std_msgs.msg import String

def serialCB(s):
  utc = datetime.datetime.utcnow() 
  if len(s.data) > 0: 
    line_parts = s.data.split('\t')
    try:
      x = float(line_parts[1])
      y = float(line_parts[2])
      theta = float(line_parts[3])
      vel = float(line_parts[5])
      omega = float(line_parts[6])
      l_count = int(line_parts[7])
      r_count = int(line_parts[8])
    except:
      return
      
  writer.writerow({'timestamp': utc, 'x': x, 'y': y, 'theta': theta, 'V': vel, 'Omega': omega, 'Left_count': l_count, 'Right_count': r_count})

# Init ROS node
rospy.init_node('log_data')

# Subcribers
serial_sub = rospy.Subscriber('serial', String, serialCB)

rate = rospy.Rate(1)

csvfile = open('logs.csv', 'w')
fieldnames = ['timestamp', 'x', 'y', 'theta', 'V', 'Omega', 'Left_count', 'Right_count']
writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
writer.writeheader()

while not rospy.is_shutdown():
  rate.sleep()
