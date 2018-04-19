import tf
import math

x= 0.00567297445607
y= -0.0544738713886
z= -0.377833004878
w= 0.92425247369



euler = tf.transformations.euler_from_quaternion((x, y, z, w))

print euler[0]*180/math.pi
print euler[1]*180/math.pi
print euler[2]*180/math.pi
