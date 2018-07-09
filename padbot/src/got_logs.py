#!/usr/bin/env python 
import rospy 
import sys
import csv 
import math

from geometry_msgs.msg import Vector3

# Class 
class CSV_log(object): 
  def __init__(self): 
    self.left_x = 0
    self.left_y = 10
    self.left_z = 0
    self.right_x = 0
    self.right_y = -10
    self.right_z = 0
  
    rospy.init_node('gamesontrack_log')
    self.rate = rospy.Rate(10) 
    if len(sys.argv) > 1:
      self.file_name = "got_" + str(sys.argv[1])
    else:
      self.file_name = 'log.csv'
    
    # Subcribers 
    rospy.Subscriber('gamesontrack/left', Vector3, self.leftCB)  
    rospy.Subscriber('gamesontrack/right', Vector3, self.rightCB)   
    
    self.csvfile = open(self.file_name, 'w') 
    fieldnames = ['timestamp', 'x', 'y', 'theta', 'left_x', 'left_y', 'left_z', 'right_x', 'right_y', 'right_z'] 
    self.writer = csv.DictWriter(self.csvfile, fieldnames=fieldnames) 
    self.writer.writeheader() 
  
  def Running(self):
    while not rospy.is_shutdown():
      t = rospy.Time.now()
      x = (self.left_x + self.right_x) / 2
      y = (self.left_y + self.right_y) / 2
      theta = math.atan2((self.right_y - self.left_y),(self.right_x - self.left_x)) + math.pi/2
      while theta > math.pi:
        theta -= math.pi * 2
      while theta < -math.pi:
        theta += math.pi * 2
      self.writer.writerow({'timestamp': t, 'x': x, 'y': y, 'theta': theta, 'left_x': self.left_x, 'left_y': self.left_y, 'left_z': self.left_z, 'right_x': self.right_x, 'right_y': self.right_y, 'right_z': self.right_z}) 
      self.rate.sleep()
  
  def Stop(self):
    self.csvfile.close()
            
  def leftCB(self, measurement): 
    self.left_x = measurement.x
    self.left_y = measurement.y
    self.left_z = measurement.z
  
  def rightCB(self, measurement): 
    self.right_x = measurement.x
    self.right_y = measurement.y
    self.right_z = measurement.z

if __name__ == '__main__':  
  log = CSV_log()
  rospy.on_shutdown(log.Stop)
  try:
    log.Running()
  except rospy.ROSInterruptException:
    log.Stop()
    
