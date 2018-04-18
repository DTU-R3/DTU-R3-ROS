import tf
import math

x= -0.0938000863251
y= -0.0260724750863
z= 0.766651709621
w= 0.634639209302


euler = tf.transformations.euler_from_quaternion((x, y, z, w))

print euler[0]*180/math.pi
print euler[1]*180/math.pi
print euler[2]*180/math.pi
