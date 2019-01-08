#!/usr/bin/env python 
import rospy
import os

from std_msgs.msg import String

# Class 
class espeak(object): 
  def __init__(self): 

    # Init ROS node
    rospy.init_node('espeak')
    self.freq = 10  # 10 Hz
    self.rate = rospy.Rate(self.freq)	

    # Subcribers 
    rospy.Subscriber('espeak', String, self.speakCB)  
  
  def Start(self):
    while not rospy.is_shutdown():
      self.rate.sleep()
            
  def speakCB(self, s): 
    os.system("espeak -ven+f3 -k5 -s150 '" + s.data +"'")

if __name__ == '__main__':  
  speaker = espeak()
  speaker.Start()
    
