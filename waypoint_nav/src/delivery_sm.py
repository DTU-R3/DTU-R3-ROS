#!/usr/bin/env python
import rospy
import json

from smach import State, StateMachine
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

# Waypoint mode, stop when last waypoint is reached
class Waypoint(State):
  def __init__(self, p_arr):
    State.__init__(self, outcomes=['success'])
    self.finished = False
    self.points = p_arr

    # Subscirber
    rospy.Subscriber('waypoint/reached', NavSatFix, self.reachCB)

  def execute(self, userdata):
    pub.pointPub(self.points[0])
    pub.statePub("RUNNING")
    while not self.finished:
      rospy.sleep(0.1)
    pub.statePub("STOP")
    return 'success'

  def reachCB(self, nat):
    index = self.points.index([nat.longitude,nat.latitude])
    if index >= (len(self.points) - 1):
      self.finished = True
    else:       
      pub.pointPub(self.points[index+1])
      pub.statePub("RUNNING")
    return

# Waypoint mode, stop when detects target fiducial
class Waypoint_fid(State):
  def __init__(self, p_arr, fid):
    State.__init__(self, outcomes=['success'])
    self.finished = False
    self.points = p_arr
    self.fid_id = fid
    self.detected_fid = 0

    # Subscirber
    rospy.Subscriber('waypoint/reached', NavSatFix, self.reachCB)
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transCB)

  def execute(self, userdata):
    pub.pointPub(self.points[0])
    pub.statePub("RUNNING")
    while not self.finished:
      if self.detected_fid == self.fid_id:
        break
      rospy.sleep(0.1)
    pub.statePub("STOP")
    return 'success'

  def reachCB(self, nat):
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

# Corridor, stop when detects target fiducial
class Corridor_fid(State):
  def __init__(self, corridor_cmd, fid):
    State.__init__(self, outcomes=['success'])
    self.cmd = corridor_cmd
    self.fid_id = fid
    self.detected_fid = 0

    # Subscirber
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.transCB)
    
  def execute(self, userdata):
    pub.modePub(self.cmd)
    while not self.fid_id == self.detected_fid:
      rospy.sleep(0.1)
    pub.modePub("STOP")
    return 'success'

  def transCB(self, t):
    if len(t.transforms) < 1:
      self.detected_fid = 0
      return
    for fid_trans in t.transforms:
      self.detected_fid = fid_trans.fiducial_id 

# Espeak, only once
class Speak(State):
  def __init__(self, speak_cmd):
    State.__init__(self, outcomes=['success'])
    self.cmd = speak_cmd

  def execute(self, userdata):
    pub.speakPub(self.cmd)
    rospy.sleep(3)
    return 'success'

# Espeak, stop when target is detected by the vision kit
class Speak_cmd(State):
  def __init__(self, speak_cmd, target_cmd):
    State.__init__(self, outcomes=['success'])
    self.cmd = speak_cmd
    self.target = target_cmd
    self.target_recived = False    

    # Subscriber
    rospy.Subscriber('mqtt/commands/vision_kit', String, self.mqttCB)

  def execute(self, userdata):
    while not self.target_recived:
      pub.speakPub(self.cmd)
      rospy.sleep(3)
    return 'success'

  def mqttCB(self, m):
    if m.data == self.target:
      self.target_recived = True
    else:
      self.target_recived = False

# Delivery class
class Delivery(object):
  def __init__(self):
    # Init ROS node
    rospy.init_node('delivery_state_machine')
    self.freq = 10
    self.rate = rospy.Rate(self.freq)  
    # Subscrber
    rospy.Subscriber('delivery/scenario', String, self.scenCB)

    # Publish waypoint parameters
    pub.parameterPub("2.0,1.0,1.0,1.0")
    pub.thresholdPub(0.2)

  def Start(self):
    while not rospy.is_shutdown():
      self.rate.sleep()

  def scenCB(self, s):
    json_data = json.loads(s.data)
    sm = StateMachine(outcomes=['success'])
    with sm:
      try:
        for task in json_data["Tasks"]:
          if task["Id"] == len(json_data["Tasks"]):
            next_state = 'success'
          else:
            next_state = str(task["Id"]+1)

          if task["Name"] == "waypoint":
            StateMachine.add(str(task["Id"]), Waypoint(task["Points"]), transitions={'success':next_state})
          elif task["Name"] == "waypoint_fid":
            StateMachine.add(str(task["Id"]), Waypoint_fid(task["Points"],task["Fid"]), transitions={'success':next_state})
          elif task["Name"] == "corridor_fid":
            StateMachine.add(str(task["Id"]), Corridor_fid(task["Command"],task["Fid"]), transitions={'success':next_state})
          elif task["Name"] == "speak":
            StateMachine.add(str(task["Id"]), Speak(task["Command"]), transitions={'success':next_state})
          elif task["Name"] == "speak_cmd":
            StateMachine.add(str(task["Id"]), Speak_cmd(task["Command"],task["Target"]), transitions={'success':next_state})
      except:
        return
    sm.execute()

if __name__ == '__main__':
  pub = Publishers()
  s = Delivery() 
  s.Start()
  
  


