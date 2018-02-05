# Robot movement control based on odometry information

The package is able to control the robot to move a certain distance or turn a certain angle based on odometry information.

## Publishers

**cmd_vel** ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

Robot speed command

**odometry_control/robot_state** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Represents robot state

## Subscribers

**odom** ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

Robot odometry information

**odometry_control/cmd** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Robot control command

## Syntax of "odometry_control/cmd"

The first part represents the movement type, the second parts define the value of the command (in meters/degrees), the third part is the robot speed. If speed is not assigned, the value will be kept as default. For example:

**Forward command**

```
fwd,1,0.5
```

**Turn command**

```
turn,90,0.5
```
