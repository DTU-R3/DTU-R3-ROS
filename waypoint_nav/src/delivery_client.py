#!/usr/bin/env python
import rospy

import actionlib
from waypoint_nav.msg import DeliveryAction, DeliveryGoal, DeliveryResult, DeliveryFeedback
from std_msgs.msg import Int32, String

class delivery_client(object):
  def __init__(self):

    # Variables
    self.goal = DeliveryGoal()
    self.goalset = False

    # Init ROS node
    rospy.init_node('delivery_action_client')
    self.client = actionlib.SimpleActionClient('delivery', DeliveryAction)
    self.client.wait_for_server()
    print "Action server starts"
    
    # Subscriber
    rospy.Subscriber('mqtt/commands/voice_kit', String, self.mqttCB)

  def Start(self):
    while not rospy.is_shutdown():
      if not self.goalset:
        continue
      self.goal.start_task = 0
      self.goal.task = 6
      self.client.send_goal(self.goal, feedback_cb = self.feedbackCB)
      self.client.wait_for_result()
      print self.client.get_result().task_status
      self.goalset = False

  def feedbackCB(self, fb):
    print fb.feedback

  def mqttCB(self, m):
    self.goal.target = m.data
    self.goalset = True
    print "Command received"

if __name__ == '__main__': 
  c = delivery_client() 
  c.Start()  
