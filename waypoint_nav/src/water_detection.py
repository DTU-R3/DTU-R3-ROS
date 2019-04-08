#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
import time

class water_detection(object):
  def __init__(self):

    # Init ROS node
    rospy.init_node('demo')    

    # Publisher
    self.cmdPub = rospy.Publisher('espeak', String, queue_size = 10)

    # Subscriber
    rospy.Subscriber('water', Bool, self.waterCB)

  def Start(self):
    while not rospy.is_shutdown():
      time.sleep(1)

  def pubCmd(self, s):
    cmdMsg = String()
    cmdMsg.data = s
    self.cmdPub.publish(cmdMsg)

  def waterCB(self, b):
    if m.data == True:
      cmdMsg = String()
      cmdMsg.data = "water detected"
      self.cmdPub.publish(cmdMsg)
      time.sleep(3)
    else:
      time.sleepI(0.5)

if __name__ == '__main__': 
  c = water_detection() 
  c.Start()  
