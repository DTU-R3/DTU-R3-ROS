#!/usr/bin/env python 
import rospy 
import sys
import csv 
import tf

from nav_msgs.msg import Odometry
 
# Class 
class CSV_log(object): 
  def __init__(self): 
    rospy.init_node('padbot_log')
    self.rate = rospy.Rate(1)
    if len(sys.argv) > 1:
      self.file_name = "padbot_" + str(sys.argv[1])
    else:
      self.file_name = 'log.csv'
    
    # Subcribers 
    rospy.Subscriber('padbot/odom', Odometry, self.OdomCB)    
    
    self.csvfile = open(self.file_name, 'w') 
    fieldnames = ['timestamp', 'x', 'y', 'theta', 'vel', 'omega'] 
    self.writer = csv.DictWriter(self.csvfile, fieldnames=fieldnames) 
    self.writer.writeheader() 
  
  def Running(self):
    while not rospy.is_shutdown():
      self.rate.sleep()
  
  def Stop(self):
    self.csvfile.close()
            
  def OdomCB(self, odom): 
    t = rospy.Time.now()
    try: 
      euler = tf.transformations.euler_from_quaternion((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w))
      x = odom.pose.pose.position.x 
      y = odom.pose.pose.position.y
      theta = euler[2]
      vel = odom.twist.twist.linear.x
      omega = odom.twist.twist.angular.z
      self.writer.writerow({'timestamp': t, 'x': x, 'y': y, 'theta': theta, 'vel': vel, 'omega': omega}) 
    except: 
      return 

if __name__ == '__main__':  
  log = CSV_log()
  rospy.on_shutdown(log.Stop)
  try:
    log.Running()
  except rospy.ROSInterruptException:
    log.Stop()
    
