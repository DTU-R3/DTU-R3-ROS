#!/usr/bin/env python 
import rospy 
import sys
import csv 
import math
import datetime

from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Twist, Pose2D
 
# Class 
class CSV_log(object): 
  def __init__(self): 
    stamp = datetime.datetime.utcnow().strftime("%a%H%M%S")
    rospy.init_node('padbot_log')
    self.rate = rospy.Rate(10)
    if len(sys.argv) > 1:
      self.file_name = "padbot_" + str(sys.argv[1]) + "_" + stamp + ".csv"
    else:
      self.file_name = "padbot_" + stamp + ".csv"
    
    self.pose = Pose2D()
    self.pose.x = 0
    self.pose.y = 0
    self.pose.theta = 0
    self.v = 0
    self.w = 0
    self.left_speed = 0
    self.right_speed = 0
    self.left_count = 0
    self.right_count = 0
    self.last_left = 0
    self.last_right = 0
    self.time = rospy.Time.now()   
    
    # Padbot parameters
    self.track_width = 0.228
    self.distance_per_count = 0.0000439
    
    # Publlishers
    self.posePub= rospy.Publisher('padbot/pose', Pose2D, queue_size = 10)
    self.resetPub= rospy.Publisher('padbot/reset_encoder', Bool, queue_size = 10)
    
    # Subcribers    
    rospy.Subscriber('padbot/left_speed', Float32, self.leftSpdCB)  
    rospy.Subscriber('padbot/right_speed', Float32, self.rightSpdCB)
    rospy.Subscriber('padbot/left_count', Int32, self.leftCB)  
    rospy.Subscriber('padbot/right_count', Int32, self.rightCB)
    
    self.csvfile = open(self.file_name, 'w') 
    fieldnames = ['timestamp', 'x', 'y', 'theta', 'vel', 'omega', 'left_count', 'right_count'] 
    self.writer = csv.DictWriter(self.csvfile, fieldnames=fieldnames) 
    self.writer.writeheader() 
  
  def Running(self):
    reset_msg = Bool()
    reset_msg.data = True
    self.resetPub.publish(reset_msg)
    rospy.sleep(1)
    while not rospy.is_shutdown():
      deltaLeft = self.left_count - self.last_left
      deltaRight = self.right_count - self.last_right
      delta_time = float(str(rospy.Time.now() - self.time))/1000000000
      self.time = rospy.Time.now() 
      self.last_left = self.left_count
      self.last_right = self.right_count
      if deltaLeft > 10000 or deltaRight > 10000:
        continue
      deltaDistance = (deltaLeft + deltaRight) * self.distance_per_count / 2
      deltaAngle = (deltaRight - deltaLeft) * self.distance_per_count / self.track_width
      self.pose.x += deltaDistance * math.cos(self.pose.theta)
      self.pose.y += deltaDistance * math.sin(self.pose.theta)
      self.pose.theta += deltaAngle
      self.posePub.publish(self.pose)
      self.v = (self.left_speed + self.right_speed) / 2
      self.w = (self.right_speed - self.left_speed) / self.track_width
      self.writer.writerow({'timestamp': self.time, 'x': self.pose.x, 'y': self.pose.y, 'theta': self.pose.theta, 'vel': self.v, 'omega': self.w, 'left_count': self.last_left, 'right_count': self.last_right}) 
      self.rate.sleep()
  
  def Stop(self):
    self.csvfile.close()
  
  def leftSpdCB(self, l):
    self.left_speed = l.data
    
  def rightSpdCB(self, r):
    self.right_speed = r.data
    
  def leftCB(self, l):
    self.left_count = l.data
    
  def rightCB(self, r):
    self.right_count = r.data

if __name__ == '__main__':  
  log = CSV_log()
  rospy.on_shutdown(log.Stop)
  try:
    log.Running()
  except rospy.ROSInterruptException:
    log.Stop()
    
