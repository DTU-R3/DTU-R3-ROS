#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String

class test_json(object):
  def __init__(self):
    # Init ROS node
    rospy.init_node('json_tester')
    
    # Publisher
    self.taskPub = rospy.Publisher('delivery/scenario', String, queue_size = 10, latch=True)

  def Start(self):
    json_data = json.load(open("test_task.json"))
    x = String()
    x.data = json.dumps(json_data)
    self.taskPub.publish(x)
    print x.data
    rospy.spin()

if __name__ == '__main__': 
  c = test_json() 
  c.Start()
