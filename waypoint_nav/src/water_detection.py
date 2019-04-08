#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import String, Bool

class water_detection(object):
  def __init__(self):

    # Init ROS node
    rospy.init_node('water_detection')    
    self.water_detected = False

    # Publisher
    self.cmdPub = rospy.Publisher('espeak', String, queue_size = 10)

    # Subscriber
    rospy.Subscriber('arlobot/water', Bool, self.waterCB)

  def Start(self):
    while not rospy.is_shutdown():
      if self.water_detected:
        cmdMsg = String()
        cmdMsg.data = "water detected"
        self.cmdPub.publish(cmdMsg)
        time.sleep(3)
      else:
        time.sleep(0.1)

  def waterCB(self, b):
    self.water_detected = b.data

if __name__ == '__main__': 
  c = water_detection() 
  c.Start()  
