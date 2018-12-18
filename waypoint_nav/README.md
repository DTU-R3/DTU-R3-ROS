# ROS-waypoint-navigation
This is the code to drive the robot through a series of waypoint defined by the user. The package includes navigation by GamesOnTrack, odometry and fiducial markers.

## Setup
Install pyproj package
```
sudo apt-get install python-pip
sudo pip install pyproj
```

## Waypoint navigation
When a waypoint is set, the robot movement is devided into two phases: turining to the waypoint and forwarding to the waypoint. For each phases, the robot follows below equations.

**While turning**
```python
vel.linear.x = 0
vel.angular.x = K_ROLL * roll
vel.angular.y = K_PITCH * pitch
vel.angular.z = K_YAW * yaw
```

**While forwarding**
```python
vel.linear.x = K_RHO * distance
vel.angular.y = K_PITCH * pitch
vel.angular.z = K_YAW * yaw
```

## Corridor mode
Corridor mode is a specific mode that enables the robot drive in the middle of the corridor based on lidar measurements. It calculates the average of 10 measurements from each side which are closest to the centerline of the robot. And then adjust the robot angular speed based on the difference of both measurements. Commands should be published to "corridor_mode"([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html)). The valid commands are:
* 'STOP': Stop the corridor mode.
* 'MID,n': Keep the robot stay in the middle of the corridor. The robot will move forward if the difference of both side is bigger than n meters.
* 'LEFT,n': Let the robot run along the left wall with a distance n meters.
* 'RIGHT,n': Let the robot run along the right wall with a distance n meters.

## Delivery scenario
The demo is designed to ask the robot fetch the object and deliver it back to the user. Several technologies are involved in this demo such as voice/vision recognition, robot control, localisation, navigation and speech synthesis. The vision and voice recognition are done by the Google AIY Vision and Voice, respectively.

### Hardware list
**[Arlobot Kit](https://learn.parallax.com/tutorials/arlo)
**[Google AIY Vision Kit](https://aiyprojects.withgoogle.com/vision)
**[Google AIY Voice Kit](https://aiyprojects.withgoogle.com/voice)
**[Raspberry Pi](https://www.raspberrypi.org/)
**[Raspberry Pi camera](https://www.raspberrypi.org/products/camera-module-v2/)
**[RPLidar](http://www.slamtec.com/en/Lidar/A3)
**Speaker

### Setup the demo on development machine
Development machine refers to the image with full source code that can be used in development. Code on Raspberry Pi can be executed by systemd so that it can automatically start when the pi boots. To set up source code, see [DTU-R3-ROS](https://github.com/DTU-R3/DTU-R3-ROS).

In order to set up the system service file, go to systemd folder under [DTU-R3-ROS](https://github.com/DTU-R3/DTU-R3-ROS), and set up repective service files based different platform. Take vision kit for example, run:'
```
sudo mv vision_demo.service /lib/systemd/system/
sudo systemctl enable vision_demo.service
sudo service vision_demo start
```

To manually stop your service, run:
```
sudo service vision_demo stop
```

To check the status of your service, run:
```
sudo service vision_demo status
```

For more details on systemd configuration, see [manual page](https://www.freedesktop.org/software/systemd/man/systemd.service.html).

### Deployment
In order to quickly reproduce the demo without too much professional knowledge, it can also be deployed on Raspberry with default Raspian image through docker.

## Launch files
**fiducial_encoder_waypoint.launch:** 3D waypoint navigation by fiducials together with encoders. The encoders are used to localise the robot while the fiducials are used to correct the transformation from odometry frame to utm frame.

**encoder_waypoint.launch:** 3D waypoint navigation by encoders only. Two waypoints are needed to intialise the robot. The first waypoint is to set the robot position in GPS frame while the second waypoint is to orientate the robot and define the destination at the same time.

**odometry_control.launch:** Higher level control based on odometry. Commands such as 'forward 1 meter' and 'turn 90 degrees'.

## 3d_waypoint_control.py

The package is able to navigate the robot in 3D.

### Publishers

**cmd_vel** ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

Robot speed command

**waypoint/robot_state** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Represents robot state

**calib_pose** ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

Used to send the robot initial position to the localisation module.

### Subscribers

**robot_gps_pose** ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

Robot GPS position

**waypoint** ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

Waypoint GPS position

**waypoint/state** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

State to enable/disable waypoint navigation

**waypoint/control_parameters** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Waypoint control parameters.

**waypoint/max_linear_speed** ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

Robot max linear speed, defaule 1.0 m/s.

**waypoint/max_angular_speed** ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

Robot max angular speed, defaule 1.0 rad/s.

**waypoint/turning_thres** ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

Threshold for the turning phase.

## encoder_waypoint_localization.py

The package is able to localise the robot by encoders.

### Publishers

**robot_gps_pose** ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

Robot GPS position

### Subscribers

**calib_pose** ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

Used to calib the transformation from odometry frame to utm frame.

**odom** ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

Robot odometry

## fiducial_waypoint_localization.py

The package is able to localise the robot by fiducials.

### Publishers

**robot_gps_pose** ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

Robot GPS position, when used to calib encoders, the topic should be mapped to "/calib_pose"

### Subscribers

**fiducial_map_GPS** ([fiducial_msgs/FiducialMapEntryArray](http://http://docs.ros.org/kinetic/api/fiducial_msgs/html/msg/FiducialMapEntryArray.html))

Update the fiducial map, the fiducial position should be in GPS frame.

**fiducial_transforms** ([nav_msgs/Odometry](http://http://docs.ros.org/kinetic/api/fiducial_msgs/html/msg/FiducialTransformArray.html))

Array of transformations from the fidicials markers to the camera.

## odometry_control.py
Higher level control based on odometry.

### Publishers

**cmd_vel** ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

Robot speed command

### Subscribers

**odom** ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

Robot odometry.

**odometry_control/cmd** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Higher level command.

**odometry_control/state** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

State to enable/disable odometry control

**odometry_control/control_parameters** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Odometry control parameters.

**odometry_control/max_linear_speed** ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

Robot max linear speed, defaule 1.0 m/s.

**odometry_control/max_angular_speed** ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

Robot max angular speed, defaule 1.0 rad/s.

**odometry_control/forwarding_thres** ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

Threshold for the forward phase.

**odometry_control/turning_thres** ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

Threshold for the turning phase.

## R3_functions.py
General functions that are used by other codes.
