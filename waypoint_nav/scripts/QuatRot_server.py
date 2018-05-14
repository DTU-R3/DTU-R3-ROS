#!/usr/bin/env python

from waypoint_nav.srv import *
from geometry_msgs.msg import Quaternion
import rospy
import tf

def handle_quat_rot(req):
    euler = tf.transformations.euler_from_quaternion((req.q.x, req.q.y, req.q.z, req.q.w))
    quat = tf.transformations.quaternion_from_euler(euler[0]+req.deg_x*math.pi/180, euler[1]+req.deg_y*math.pi/180, euler[2]+req.deg_z*math.pi/180)
    result = Quaternion()
    result.x = quat[0]
    result.y = quat[1]
    result.z = quat[2]
    result.w = quat[3]
    return QuatRotResponse(result)

def QuatRot_server():
    rospy.init_node('QuatRot_server')
    s = rospy.Service('QuatRot', QuatRot, handle_quat_rot)
    rospy.spin()

if __name__ == "__main__":
    QuatRot_server()
