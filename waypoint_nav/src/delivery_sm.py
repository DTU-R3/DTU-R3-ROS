#!/usr/bin/env python
import rospy
import json

from std_msgs.msg import String, Bool, Int32, Float32
from sensor_msgs.msg import NavSatFix
from fiducial_msgs.msg import FiducialTransformArray

# Publishers class
class Publishers(object):
  def __init__(self):
    # Publisher
    self.waypoint_statePub = rospy.Publisher('waypoint/state', String, queue_size = 10)
    self.waypointPub = rospy.Publisher('waypoint', NavSatFix, queue_size = 10)
    self.corridorPub = rospy.Publisher('corridor_mode', String, queue_size = 10)
    self.espeakPub = rospy.Publisher('espeak', String, queue_size = 10)
    self.thresPub = rospy.Publisher('waypoint/forwarding_thres', Float32, queue_size = 10)
    self.paramPub = rospy.Publisher('waypoint/control_parameters', String, queue_size = 10)
    self.cmdPub = rospy.Publisher('delivery/cmd', String, queue_size = 10)

  def statePub(self, s):
    stateMsg = String()
    stateMsg.data = s
    self.waypoint_statePub.publish(stateMsg)

  def pointPub(self, p):
    waypointMsg = NavSatFix()
    waypointMsg.longitude = p[0]
    waypointMsg.latitude = p[1]
    waypointMsg.altitude = 0
    self.waypointPub.publish(waypointMsg)

  def modePub(self, s):
    modeMsg = String()
    modeMsg.data = s
    self.corridorPub.publish(modeMsg)

  def speakPub(self, s):
    speakMsg = String()
    speakMsg.data = s
    self.espeakPub.publish(speakMsg)

  def thresholdPub(self, f):
    thresMsg = Float32()
    thresMsg.data = f
    self.thresPub.publish(thresMsg)

  def parameterPub(self, s):
    paramMsg = String()
    paramMsg.data = s
    self.paramPub.publish(paramMsg)

  def commandPub(self, s):
    cmdMsg = String()
    cmdMsg.data = s
    self.cmdPub.publish(cmdMsg)

# Waypoint mode, stop when last waypoint is reached
class Waypoint(object):
  def __init__(self):
    self.finished = False
    self.stop = False
    self.points = []

    # Subscirber
    rospy.Subscriber('waypoint/reached', NavSatFix, self.reachCB)
    rospy.Subscriber('delivery/cmd', String, self.cmdCB)

  def execute(self):
    print self.points
    pub.pointPub(self.points[0])
    pub.statePub("RUNNING")
    while not self.finished:
      if self.stop:
        self.stop = False
        pub.statePub("STOP")
        return False
      rospy.sleep(0.1)
    pub.statePub("STOP")
    return True

  def reachCB(self, nat):
    print [nat.longitude,nat.latitude]
    print self.points
    index = self.points.index([nat.longitude,nat.latitude])
    if index >= (len(self.points) - 1):
      self.finished = True
    else:       
      pub.pointPub(self.points[index+1])
      pub.statePub("RUNNING")
    return

  def cmdCB(self, s):
    if s.data == "STOP":
      self.stop = True

# Waypoint mode, stop when detects target fiducial
class Waypoint_fid(object):
  def __init__(self):
    self.finished = False
    self.stop = False
    self.points = []
    self.fid_id = 0
    self.detected_fid = 0

    # Subscirber
    rospy.Subscriber('waypoint/reached', NavSatFix, self.reachCB)
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transCB)
    rospy.Subscriber('delivery/cmd', String, self.cmdCB)

  def execute(self):
    print self.points
    self.detected_fid = 0
    pub.pointPub(self.points[0])
    pub.statePub("RUNNING")
    while not self.finished:
      if self.stop:
        self.stop = False
        pub.statePub("STOP")
        return False
      if self.detected_fid == self.fid_id:
        print self.detected_fid
        break
      rospy.sleep(0.1)
    self.detected_fid = 0
    pub.statePub("STOP")
    return True

  def reachCB(self, nat):
    print [nat.longitude,nat.latitude]
    print self.points
    index = self.points.index([nat.longitude,nat.latitude])
    if index >= (len(self.points) - 1):
      self.finished = True
    else:       
      pub.pointPub(self.points[index+1])
      pub.statePub("RUNNING")
    return

  def transCB(self, t):
    if len(t.transforms) < 1:
      self.detected_fid = 0
      return
    for fid_trans in t.transforms:
      self.detected_fid = fid_trans.fiducial_id  

  def cmdCB(self, s):
    if s.data == "STOP":
      self.stop = True

# Corridor, stop when detects target fiducial
class Corridor_fid(object):
  def __init__(self):
    self.stop = False
    self.cmd = "STOP"
    self.fid_id = 0
    self.detected_fid = 0

    # Subscirber
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transCB)
    rospy.Subscriber('delivery/cmd', String, self.cmdCB)
    
  def execute(self):
    pub.modePub(self.cmd)
    while not self.fid_id == self.detected_fid:
      if self.stop:
        self.stop = False
        pub.modePub("STOP")
        return False
      rospy.sleep(0.1)
    self.detected_fid = 0
    pub.modePub("STOP")
    return True

  def transCB(self, t):
    if len(t.transforms) < 1:
      self.detected_fid = 0
      return
    for fid_trans in t.transforms:
      self.detected_fid = fid_trans.fiducial_id 

  def cmdCB(self, s):
    if s.data == "STOP":
      self.stop = True

# Espeak, only once
class Speak(object):
  def __init__(self):
    self.cmd = ""

  def execute(self):
    pub.speakPub(self.cmd)
    rospy.sleep(3)
    return True

# Espeak, stop when target is detected by the vision kit
class Speak_cmd(object):
  def __init__(self):
    self.cmd = ""
    self.target = ""
    self.target_recived = False    
    self.stop = False

    # Subscriber
    rospy.Subscriber('mqtt/commands/vision_kit', String, self.mqttCB)
    rospy.Subscriber('delivery/cmd', String, self.cmdCB)

  def execute(self):
    while not self.target_recived:
      if self.stop:
        self.stop = False
        return False
      pub.speakPub(self.cmd)
      rospy.sleep(3)
    return True

  def mqttCB(self, m):
    if m.data == self.target:
      self.target_recived = True
    else:
      self.target_recived = False

  def cmdCB(self, s):
    if s.data == "STOP":
      self.stop = True

# Delivery class
class Delivery(object):
  def __init__(self):
    # Init ROS node
    rospy.init_node('delivery_state_machine')
    self.freq = 10
    self.rate = rospy.Rate(self.freq)  
    self.class_init = False
    self.json_data = ""

    # Subscrber
    rospy.Subscriber('delivery/scenario', String, self.scenCB)

    # Publish waypoint parameters
    pub.parameterPub("2.0,1.0,1.0,1.0")
    pub.thresholdPub(0.2)

  def Start(self):
    while not rospy.is_shutdown():
      if not self.class_init:
        self.rate.sleep()
        continue
      try:
        for task in self.json_data["Tasks"]:
          if task["Name"] == "waypoint":
            c = Waypoint()
            c.points = task["Points"]
          elif task["Name"] == "waypoint_fid":
            c = Waypoint_fid()
            c.points = task["Points"]
            c.fid_id = task["Fid"]
          elif task["Name"] == "corridor_fid":
            c = Corridor_fid()
            c.cmd = task["Command"]
            c.fid_id = task["Fid"]
          elif task["Name"] == "speak":
            c = Speak()
            c.cmd = task["Command"]
          elif task["Name"] == "speak_cmd":
            c = Speak_cmd()
            c.cmd = task["Command"]
            c.target = task["Target"]
          print c
          if not c.execute():
            break
      except:
        continue
      self.class_init = False
      self.rate.sleep()

  def Stop(self):
    pub.modePub("STOP")
    pub.statePub("STOP")

  def scenCB(self, s):
    if self.class_init:
      pub.commandPub("STOP")
      while self.class_init:
        self.rate.sleep() 
    self.json_data = json.loads(s.data)
    self.class_init = True

if __name__ == '__main__':
  pub = Publishers()
  s = Delivery() 
  s.Start()
  
  


