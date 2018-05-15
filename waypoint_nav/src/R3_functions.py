#!/usr/bin/env python
def quat_rot(q, deg_x, deg_y, deg_z):
    euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    quat = tf.transformations.quaternion_from_euler(euler[0]+math.radians(deg_x), euler[1]+math.radians(deg_y), euler[2]+math.radians(deg_z))
    quat = Quaternion()
    quat.x = quat[0]
    quat.y = quat[1]
    quat.z = quat[2]
    quat.w = quat[3]
    return quat

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
