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
    self.started = False
    self.command_list = ["start", "stop", "pause"]

    # Init ROS node
    rospy.init_node('delivery_action_client')
    
    # Publisher
    self.fbPub = rospy.Publisher('delivery/output', String, queue_size = 10)
    self.cmdPub = rospy.Publisher('delivery/command', String, queue_size = 10)
    self.espeakPub = rospy.Publisher('espeak', String, queue_size = 10)

    # Subscriber
    rospy.Subscriber('mqtt/commands/voice_kit', String, self.mqttCB)

    # Start action client
    self.client = actionlib.SimpleActionClient('delivery', DeliveryAction)
    self.client.wait_for_server()
    self.speakPub("Action server started")
    self.feedbackPub("Action server started")
    print "Action server started"
    self.started = True

  def Start(self):
    while not rospy.is_shutdown():
      if not self.goalset:
        continue
      self.goal.start_task = 0
      self.goal.task = 6
      self.client.send_goal(self.goal, feedback_cb = self.feedbackCB)
      self.goalset = False

  def feedbackCB(self, fb):
    self.feedbackPub(fb.feedback)

  def mqttCB(self, m):
    if not self.started:
      self.speakPub("Action server has not started")
    if m.data in self.command_list:
      self.commandPub(m.data)
      return
    self.goal.target = m.data
    self.goalset = True
    self.feedbackPub("Command received")
  
  def feedbackPub(self, s):
    fbMsg = String()
    fbMsg.data = s
    self.fbPub.publish(fbMsg)

  def commandPub(self, s):
    cmdMsg = String()
    cmdMsg.data = s
    self.cmdPub.publish(cmdMsg)

  def speakPub(self, s):
    speakMsg = String()
    speakMsg.data = s
    self.espeakPub.publish(speakMsg)

if __name__ == '__main__': 
  c = delivery_client() 
  c.Start()  
