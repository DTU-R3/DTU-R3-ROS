#!/usr/bin/env python 
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Class 
class alex_mqtt(object): 
  def __init__(self): 
    # Init ROS node
    rospy.init_node('alex_mqtt')
    self.freq = 10  # 10 Hz
    self.rate = rospy.Rate(self.freq)	
    self.vel = Twist()
    
    # Publishers
    self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    # Subcribers 
    rospy.Subscriber('iot/commands/control/test', String, self.mqttCB)  
  
  def Start(self):
    while not rospy.is_shutdown():
      self.rate.sleep()
            
  def mqttCB(self, str): 
    if str.data == "START":
      self.vel.linear.x = 0.5
    else:
      self.vel.linear.x = 0
    self.vel_pub.publish(self.vel)
    
if __name__ == '__main__':  
  ctrl = alex_mqtt()
  ctrl.Start()
    
