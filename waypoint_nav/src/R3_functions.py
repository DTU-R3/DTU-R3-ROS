#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

def quat_rot(q, deg_x, deg_y, deg_z):
    euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    quat = tf.transformations.quaternion_from_euler(euler[0]+math.radians(deg_x), euler[1]+math.radians(deg_y), euler[2]+math.radians(deg_z))
    res = Quaternion()
    res.x = quat[0]
    res.y = quat[1]
    res.z = quat[2]
    res.w = quat[3]
    return res

def fit_in_rad(r):
  while r > math.pi:
    r = r - 2 * math.pi
  while r < -math.pi:
    r = r + 2 * math.pi
  return r
  
def debug_info(pub, info):
  s = String()
  s.data = info
  pub.publish(s)
