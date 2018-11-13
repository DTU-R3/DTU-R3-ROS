#!/usr/bin/env python
import rospy

import actionlib
from waypoint_nav.msg import DeliveryAction, DeliveryGoal, DeliveryResult, DeliveryFeedback
from std_msgs.msg import String, Bool
from sensor_msgs.msg import NavSatFix
from fiducial_msgs.msg import FiducialTransformArray

class delivery_server(object):
  def __init__(self):

    # Variables
    self.tasks = ["Go to corridor", "Navigate in corridor", "Enter logistic room", "Wait for load", "Exit logistic room", "Back in corridor", "Back to Office"]
    self.detected_fid = 0
    self.office_corridor = []
    self.corridor_logistic = []

    # Init ROS node
    rospy.init_node('delivery_action_server')
    self.freq = 10
    self.rate = rospy.Rate(self.freq)

    # ROS Action server
    self.server = actionlib.SimpleActionServer('delivery', DeliveryAction, self.do_delivery, False) 
    
    # Publishers
    self.waypoint_statePub = rospy.Publisher('waypoint/state', String, queue_size = 10)
    self.waypointPub = rospy.Publisher('waypoint', NavSatFix, queue_size = 10)
    self.corridorPub = rospy.Publisher('corridor_mode', Bool, queue_size = 10)

    # Subscribers
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transCB)
    rospy.Subscriber('waypoint/reached', Int32, self.reachCB)
    rospy.Subscriber('robot_gps_pose', Odometry, self.poseCB)

  def Start(self):
    self.server.start()
    rospy.spin()

  def do_delivery(self, goal):
  
    if goal.task > len(self.tasks):
      result = DeliveryResult()
      result.task_status = "Task not find"
      self.server.set_aborted(result, "Task undefined")
      return
  
    current_task = 0
    while current_task < len(self.tasks):
      if self.server.is_preempt_requested():
        result = DeliveryResult()
        result.task_status = self.tasks[current_task]
        self.server.set_preempted(result, "Task preempted")
        return
    
      feedback = DeliveryFeedback()
      feedback.current_task = self.tasks[current_task]
      self.server.publish_feedback(feedback)
      self.rate.sleep()

      ### Carry out the task ###
      # From office to corridor    
      if current_task == 0:
        current_task += 1
        continue

      # Corridor mode, to logistic room  
      if current_task == 1:
        current_task += 1
        continue

      # Enter logistic room    
      if current_task == 2:
        current_task += 1
        continue
    
      # Wait for load    
      if current_task == 3:
        current_task += 1
        continue

      # Exit logistic room   
      if current_task == 4:
        current_task += 1
        continue
    
      # Corridor mode, to office    
      if current_task == 5:
        current_task += 1
        continue

      # Enter office  
      if current_task == 6:
        current_task += 1
        continue

    result = DeliveryResult()
    result.task_status = "Task Completed"
    self.server.set_succeeded(result, "Delivery Completed")

if __name__ == '__main__': 
  s = delivery_server() 
  s.Start()

