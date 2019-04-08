#!/usr/bin/env python
import rospy
import time
import math

from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

class water_demo(object):
  def __init__(self):

    # Init ROS node
    rospy.init_node('water_demo')

    self.INIT = 0
    self.FIND_WATER = 1
    self.TURN = 2
    self.BACK = 3
    self.DONE = 4

    self.task = self.INIT
    self.cmd = "STOP"
    self.odom_detected = False
    self.init_odom = Odometry()
    self.odom = Odometry() 

    # Publisher
    self.corridorPub = rospy.Publisher('corridor_mode', String, queue_size = 10)
    self.ctrlPub = rospy.Publisher('odometry_control/cmd', String, queue_size = 10)
    self.espeakPub = rospy.Publisher('espeak', String, queue_size = 10)

    # Subscriber
    rospy.Subscriber('water_demo', String, self.cmdCB)
    rospy.Subscriber('arlobot/water', Bool, self.waterCB)
    rospy.Subscriber('odom', Odometry, self.odomCB)
    rospy.Subscriber('odometry_control/finished', Bool, self.controlCB)

  def Start(self):
    while not rospy.is_shutdown():
      if not self.cmd == "START":
        time.sleep(1)
        continue
      if not self.odom_detected:
        time.sleep(3)
        continue

      if self.task == self.INIT:
        self.water_detected = False
        self.turn_finished = False
        self.init_odom = self.odom
        self.distance = 0
        self.modePub("MID,0.4")
        self.task = self.FIND_WATER
      elif self.task == self.FIND_WATER:
        if self.water_detected:
          self.modePub("STOP")
          time.sleep(1)
          self.speakPub("water detected")
          time.sleep(3)
          odo = self.odom
          self.distance = self.getDistance(odo, self.init_odom)
          self.controlPub("turn,180")
          self.task = self.TURN
      elif self.task == self.TURN:
        if self.turn_finished:
          self.init_odom = self.odom
          self.speakPub("return to base")
          time.sleep(3)
          self.modePub("MID,0.4")
          self.task = self.BACK
      elif self.task == self.BACK:
        odo = self.odom
        dis = self.getDistance(odo, self.init_odom)
        print "---"
        print dis
        print self.distance
        if dis > self.distance:
          self.modePub("STOP")
          time.sleep(1)
          self.task = self.DONE
      elif self.task == self.DONE:
        self.cmd = "STOP"
        self.task = self.INIT
        self.speakPub("task completed")
        time.sleep(3)
      time.sleep(0.1)

  def getDistance(self, first_odo, sec_odo):
    dx = first_odo.pose.pose.position.x - sec_odo.pose.pose.position.x
    dy = first_odo.pose.pose.position.y - sec_odo.pose.pose.position.y
    dis = math.sqrt( dx**2 + dy**2 )
    return dis

  def modePub(self, s):
    modeMsg = String()
    modeMsg.data = s
    self.corridorPub.publish(modeMsg)
  
  def controlPub(self, s):
    cmdMsg = String()
    cmdMsg.data = s
    self.ctrlPub.publish(cmdMsg)

  def speakPub(self, s):
    cmdMsg = String()
    cmdMsg.data = s
    self.espeakPub.publish(cmdMsg)

  def cmdCB(self, s):
    self.cmd = s.data
    if self.cmd == "PAUSE":
      self.modePub("STOP")
    if self.cmd == "STOP":
      self.modePub("STOP")
      self.task = self.INIT

  def waterCB(self, b):
    self.water_detected = b.data

  def odomCB(self, odo):
    self.odom_detected = True
    self.odom = odo

  def controlCB(self, b):
    self.turn_finished = b.data

if __name__ == '__main__': 
  c = water_demo() 
  c.Start()  
