#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String

class demo(object):
  def __init__(self):

    # Init ROS node
    rospy.init_node('demo')

    # Variables
    self.target = "" 
    self.task = "" 
    self.ready = False   

    # rosparams
    self.task_file = rospy.get_param("~demo/task_file", "tasks.json")     

    # Publisher
    self.taskPub = rospy.Publisher('delivery/scenario', String, queue_size = 10, latch=True)
    self.cmdPub = rospy.Publisher('delivery/cmd', String, queue_size = 10)

    # Subscriber
    rospy.Subscriber('mqtt/commands/voice_kit', String, self.mqttCB)
    rospy.Subscriber('demo/command', String, self.demoCB)
    rospy.Subscriber('demo/tasks', String, self.tasksCB)

  def Start(self):
    json_tasks = json.load(open(self.task_file))
    self.task = json.dumps(json_tasks)
    rospy.sleep(3)
    while not rospy.is_shutdown():
      if self.ready:
        self.pubTask(self.task)
        self.ready = False
      rospy.sleep(1)

  def ChangeTarget(self):
    data_json = json.loads(self.task)
    tasks = []
    for task in data_json["Tasks"]:
      if task["Name"] == "speak_cmd":
        task["Command"] = self.target + " please"
        task["Target"] = self.target
      tasks.append(task)
    self.task = json.dumps({"Tasks": tasks})

  def pubTask(self, s):
    taskMsg = String()
    taskMsg.data = s
    self.taskPub.publish(taskMsg)

  def pubCmd(self, s):
    cmdMsg = String()
    cmdMsg.data = s
    self.cmdPub.publish(cmdMsg)

  def mqttCB(self, m):
    if m.data == "stop":
      self.pubCmd("STOP") 
    self.target = m.data
    self.ChangeTarget()
    self.ready = True
  
  def demoCB(self, s):
    if s.data == "START":
      self.ready = True
    else:
      self.pubCmd(s.data) 

  def tasksCB(self, s):
    self.task = s.data

if __name__ == '__main__': 
  c = demo() 
  c.Start()  
