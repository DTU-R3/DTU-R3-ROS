#!/usr/bin/env python

from waypoint_nav.srv import *
import rospy
import math

def handle_fit_in_rad(req):
  while req.r > math.pi:
    req.r = req.r - 2 * math.pi
  while req.r < -math.pi:
    req.r = req.r + 2 * math.pi
  return FitInRadResponse(req.r)

def FitInRad_server():
  rospy.init_node('FitInRad_server')
  s = rospy.Service('FitInRad', FitInRad, handle_fit_in_rad)
  rospy.spin()

if __name__ == "__main__":
    FitInRad_server()
