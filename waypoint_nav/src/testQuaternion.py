import tf
import math

x= -0.0874559804606
y= -0.995857553449
z= 0.020493832834
w= 0.0141133814589




euler = tf.transformations.euler_from_quaternion((x, y, z, w))

print euler[0]*180/math.pi
print euler[1]*180/math.pi
print euler[2]*180/math.pi
