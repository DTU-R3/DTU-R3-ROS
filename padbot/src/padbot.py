#!/usr/bin/env python
import rospy
import tf
import math

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from SerialDataGateway import SerialDataGateway

class padbot_serial(object):
  def __init__(self):
    self.leftCount = 0
    self.rightCount = 0
    self.x = 0
    self.y = 0
    self.theta = 0
    
    rospy.init_node('padbot')
    self.rate = rospy.Rate(1)
    self.t = rospy.Time.now()
       
    # ROS param
    self.track_width = float(rospy.get_param("~driveGeometry/trackWidth", "0.35"))
    self.distance_per_count = float(rospy.get_param("~driveGeometry/distancePerCount", "0.00025"))
    self.speed_to_cmd = rospy.get_param("~speedToCmd", 50.0)
    self.min_power = rospy.get_param("~min_power", 60)
    self.fwd_left_to_right = rospy.get_param("~fwdLeftToright", 1.06)
    self.back_left_to_right = rospy.get_param("~backLeftToright", 1.00)
    # Start serial port
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud_rate = int(rospy.get_param("~baudRate", 115200))
    self._SerialDataGateway = SerialDataGateway(port, baud_rate, self.serial_received)
        
    # Subscriptions
    rospy.Subscriber("cmd_vel", Twist, self.velCB)  # Is this line or the below bad redundancy?
    rospy.Subscriber("padbot/init_encoder", String, self.encoderCB)

    # Publishers
    self.serialPub = rospy.Publisher('serial', String, queue_size=10)
    self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
    
  def serial_received(self, line):  # This is Propeller specific
    self.serialPub.publish("In: " + line)
    if len(line) > 0:
      line_parts = line.split(',')
      if line_parts[0] == 'o':
        self.odometry_broadcast(line_parts)
        return

  def reset_serial(self):
    try:
      self._SerialDataGateway.Stop()
    except AttributeError:
      rospy.loginfo("Attempt to start nonexistent Serial device.")
      rospy.loginfo("Serial Data Gateway stopped.")
      rospy.loginfo("5 second pause to let Activity Board settle after serial port reset . . .")
      time.sleep(5)  # Give it time to settle.
      self.startSerialPort()

  def odometry_broadcast(self, line_parts):
    parts_count = len(line_parts)
    if parts_count < 3:
      rospy.logwarn("Short line from Teensy: " + str(parts_count))
      return
      
    try:
      currentLeftCount = int(line_parts[1])
      currentRightCount = int(line_parts[2])
      deltaLeft = currentLeftCount - self.leftCount
      deltaRight = currentRightCount - self.rightCount
      delta_t = float(str(rospy.Time.now() - self.t))/1000000000
      self.t = rospy.Time.now() 
      self.leftCount = currentLeftCount
      self.rightCount = currentRightCount
      # handle overflow
      if deltaLeft > 10000 or deltaRight > 10000:
        self.serial_write("o,0,0")
        self.serialPub.publish("Out: o,0,0")
        return     
      deltaDistance = (deltaLeft + deltaRight) * self.distance_per_count / 2
      deltaAngle = (deltaRight - deltaLeft) * self.distance_per_count / self.track_width
      self.x += deltaDistance * math.cos(self.theta)
      self.y += deltaDistance * math.sin(self.theta)
      self.theta += deltaAngle
      vx = deltaDistance / delta_t
      omega = deltaAngle / delta_t      
    except ValueError:
      rospy.logwarn("Bad int:")
      return

    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(self.theta / 2.0)
    quaternion.w = math.cos(self.theta / 2.0)

    odometry = Odometry()
    odometry.header.frame_id = "odom"
    odometry.header.stamp = self.t
    odometry.pose.pose.position.x = self.x
    odometry.pose.pose.position.y = self.y
    odometry.pose.pose.position.z = 0
    odometry.pose.pose.orientation = quaternion
    odometry.child_frame_id = "baselink_footprint"
    odometry.twist.twist.linear.x = vx
    odometry.twist.twist.linear.y = 0
    odometry.twist.twist.angular.z = omega
    self.odomPub.publish(odometry)

  def serial_write(self, message):
    self.serialPub.publish("Out: " + message)
    self._SerialDataGateway.Write(message)
        
  def start(self):
    rospy.loginfo("Serial Data Gateway starting . . .")
    try:
      self._SerialDataGateway.Start()
    except:
      rospy.loginfo("ARLO SERIAL PORT Start Error")
    rospy.loginfo("Serial Data Gateway started.")
    # init encoders
    self.serial_write("o,0,0")
    self.serialPub.publish("Out: o,0,0")
    while not rospy.is_shutdown():
      self.rate.sleep()

  def velCB(self, twist_command):  # This is Propeller specific
    v = twist_command.linear.x  # m/s
    omega = twist_command.angular.z  # rad/s
    max_speed = (200.0-self.min_power) / self.speed_to_cmd
    if v > 0 and v+math.fabs(omega)*self.track_width*0.5 > max_speed:
      v = max_speed - math.fabs(omega)*self.track_width*0.5
    if v < 0 and -v+math.fabs(omega)*self.track_width*0.5 > max_speed:
      v = -max_speed + math.fabs(omega)*self.track_width*0.5
        
    leftSpeed = v - omega * 0.5 * self.track_width
    rightSpeed = v + omega * 0.5 * self.track_width
      
    if leftSpeed > 0:
      l_cmd = int(self.fwd_left_to_right * ((leftSpeed * self.speed_to_cmd) + self.min_power));
    elif leftSpeed < 0:
      l_cmd = int(self.back_left_to_right * ((leftSpeed * self.speed_to_cmd) - self.min_power));
    else:
      l_cmd = 0
           
    if rightSpeed > 0:
      r_cmd = int((rightSpeed * self.speed_to_cmd) + self.min_power);
    elif rightSpeed < 0:
      r_cmd = int((rightSpeed * self.speed_to_cmd) - self.min_power);
    else:
      r_cmd = 0
        
    message = "s,%d,%d" % (l_cmd, r_cmd)
    self.serial_write(message)
    self.serialPub.publish("Out: " + message)
    
  def encoderCB(self, string_cmd):
    message = string_cmd.data
    self.serial_write(message)
    self.serialPub.publish("Out: " + message)


if __name__ == '__main__':
  robot = padbot_serial()
  robot.start()

