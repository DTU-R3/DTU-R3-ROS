#!/usr/bin/env python
import rospy

import actionlib
from waypoint_nav.msg import DeliveryAction, DeliveryGoal, DeliveryResult, DeliveryFeedback
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransformArray

class delivery_server(object):
  def __init__(self):

    # Variables
    self.tasks = ["Go to corridor", "Navigate in corridor", "Enter logistic room", "Exit logistic room", "Back in corridor", "Back to Office"]
    self.detected_fid = 0
    self.current_task = 0
    self.robot_pose = [0,0]
    self.office_corridor = [[12.5863292679,55.6617404801],[12.5862673177,55.6617452098],[12.5862676492,55.6617671639]]
    self.corridor_logistic = [[12.5863292679,55.6617404801],[12.5862673177,55.6617452098],[12.5862676492,55.6617671639]]
    self.logistic_corridor = [[12.5863292679,55.6617404801],[12.5862673177,55.6617452098],[12.5862676492,55.6617671639]]
    self.corridor_office = [[12.5863292679,55.6617404801],[12.5862673177,55.6617452098],[12.5862676492,55.6617671639]]

    # Init ROS node
    rospy.init_node('delivery_action_server')
    self.freq = 10
    self.rate = rospy.Rate(self.freq)

    # ROS Action server
    self.server = actionlib.SimpleActionServer('delivery', DeliveryAction, self.do_delivery, False) 
    self.result = DeliveryResult()    
    self.feedback = DeliveryFeedback()

    # Publishers
    self.waypoint_statePub = rospy.Publisher('waypoint/state', String, queue_size = 10)
    self.waypointPub = rospy.Publisher('waypoint', NavSatFix, queue_size = 10)
    self.corridorPub = rospy.Publisher('corridor_mode', String, queue_size = 10)
    self.speakPub = rospy.Publisher('espeak', String, queue_size = 10)

    # Subscribers
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transCB)
    rospy.Subscriber('robot_gps_pose', Odometry, self.poseCB)
    rospy.Subscriber('waypoint/reached', Int32, self.reachCB)
    rospy.Subscriber('/delivery/stop', Bool, self.stopCB) 

  def Start(self):
    self.server.start()
    rospy.spin()

  def do_delivery(self, goal):
  
    if goal.task > len(self.tasks):
      self.result.task_status = "Task not find"
      self.server.set_aborted(self.result, "Task undefined")
      return
  
    self.current_task = goal.start_task
    # Set the first waypoint to drive the robot
    if self.current_task == 0:
      self.pointPub(self.office_corridor[0])
      self.statePub("RUNNING")
      self.feedbackPub("Task 1: Moving out of the office")

    # Start the delivery tasks
    while self.current_task < goal.task:
      self.feedbackPub(self.tasks[self.current_task])
      self.rate.sleep()
      if self.server.is_preempt_requested():
        self.result.task_status = self.tasks[self.current_task]
        self.StopRobot()
        self.server.set_preempted(self.result, "Task preempted")
        return

      ### Carry out the task ###
      # From office to corridor    
      if self.current_task == 0:
        # If fiducial 208 is seen
        if self.detected_fid == 209:
          self.statePub("STOP")
          self.modePub("LEFT,1")
          self.current_task = 1
          self.feedbackPub("Task 2: corridor mode to logistic room")
        continue

      # Corridor mode, to logistic room  
      if self.current_task == 1:
        # If fiducial 209 is seen
        if self.detected_fid == 208:
          self.current_task = 2
          self.modePub("STOP")
          self.pointPub(self.corridor_logistic[0])
          self.statePub("RUNNING")
          self.feedbackPub("Task 3: Move to logistic room")
        continue

      # Enter logistic room and wait for load
      if self.current_task == 2:
        # If fiducial 210 is seen
        if self.detected_fid == 210:
          self.current_task = 3
          self.modePub("STOP")
          rospy.sleep(1)
          self.pointPub(self.logistic_corridor[0])
          self.statePub("RUNNING")
          self.feedbackPub("Task 4: Back to corridor")
        continue

      # Exit logistic room   
      if self.current_task == 3:
        # If fiducial 209 is seen
        if self.detected_fid == 208:
          self.current_task = 4
          self.statePub("STOP")
          self.modePub("RIGHT,1")
          self.feedbackPub("Task 5: corridor mode to office")
        continue
    
      # Corridor mode, to office    
      if self.current_task == 4:
        # If fiducial 208 is seen
        if self.detected_fid == 209:
          self.current_task = 5
          self.modePub("STOP")
          self.pointPub(self.corridor_office[0])
          self.statePub("RUNNING")
          self.feedbackPub("Task 6: back to the office")
        continue
      
      if self.current_task == 5:
        # If fiducial 201 is seen
        if self.detected_fid == 201:
          self.current_task = 6
          self.StopRobot()
        continue

    self.StopRobot()
    self.result.task_status = "Task Completed"
    self.server.set_succeeded(self.result, "Delivery Completed")
 
  def transCB(self, t):
    for fid_trans in t.transforms:
      self.detected_fid = fid_trans.fiducial_id

  def poseCB(self, p):
    self.robot_pose = [p.pose.pose.position.x, p.pose.pose.position.y]    

  def reachCB(self, i):
    if self.current_task == 0:
      index = self.GetClosestWaypoint(self.robot_pose, self.office_corridor)
      if index >= (len(self.office_corridor) - 1):
        self.current_task = 1
        self.statePub("STOP")
        self.modePub("LEFT,1")
      else:       
        self.pointPub(self.office_corridor[index+1])
        self.statePub("RUNNING")
      return

    if self.current_task == 2:
      index = self.GetClosestWaypoint(self.robot_pose, self.corridor_logistic)
      if index < (len(self.corridor_logistic) - 1):
        self.pointPub(self.corridor_logistic[index+1])
        self.statePub("RUNNING")
      return

    if self.current_task == 3:
      index = self.GetClosestWaypoint(self.robot_pose, self.logistic_corridor)
      if index >= (len(self.logistic_corridor) - 1):
        self.current_task == 4
        self.statePub("STOP")
        self.modePub("RIGHT,1")
      else:       
        self.pointPub(self.logistic_corridor[index+1])
        self.statePub("RUNNING")
      return
        
    if self.current_task == 5:
      index = self.GetClosestWaypoint(self.robot_pose, self.corridor_office)
      if index < (len(self.corridor_office) - 1):
        self.pointPub(self.corridor_office[index+1])
        self.statePub("RUNNING")
      else:
        self.StopRobot()
      return

  def stopCB(self, s):
    if s.data:
      self.current_task = 100
      self.StopRobot() 
      self.result.task_status = "Task stopped"
      self.server.set_succeeded(self.result, "Delivery Completed")

  def statePub(self, s):
    stateMsg = String()
    stateMsg.data = s
    self.waypoint_statePub.publish(stateMsg)

  def modePub(self, b):
    modeMsg = Bool()
    modeMsg.data = b
    self.corridorPub.publish(modeMsg)

  def pointPub(self, p):
    waypointMsg = NavSatFix()
    waypointMsg.longitude = p[0]
    waypointMsg.latitude = p[1]
    waypointMsg.altitude = 0
    self.waypointPub.publish(waypointMsg)

  def speakPub(self, s):
    speakMsg = String()
    speakMsg.data = s
    self.speakPub.publish(stateMsg)

  def feedbackPub(self, s):
    self.feedback.feedback = s
    self.server.publish_feedback(self.feedback)

  def StopRobot(self):
    self.statePub("STOP")
    self.modePub("STOP")

  def GetClosestWaypoint(self, p, p_arr):
    res = 0
    min_dis = 259201.0 # The biggest possible value
    for i in range(0,len(p_arr)-1):
      dis = (p_arr[i][0] - p[0])**2 + (p_arr[i][1] - p[1])**2
      if dis < min_dis:
        res = i
        min_dis = dis
    return res        

if __name__ == '__main__': 
  s = delivery_server() 
  s.Start()

