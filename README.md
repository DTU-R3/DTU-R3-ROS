# ROS Package for DTU-R3

## Features
* Propeller code for different robot (Power wheelchair, Padbot, Arlobot)
* Simulation of virtual robots and sensors.
* Uses Mazemap data to generate 3D floorplans for waypoint navigation and simulation.
* Can use Virtual Reality for remote control of telerobots.
* All functions can also be programmed in [Nodered](https://nodered.org/)

## Prerequisites 
ROS Kinetic

## ROS Installation
For tutorials and documentation on installing see [ROS Website](http://www.ros.org/install/)

## Project contents
The projects contains ROS packages for robot features as well as package developed by DTU-R3. Detail information for each packages locates in respective folder. 

## ROS graph of the scenrio
![ROS graph](/docs/rosgraph.png "ROS graph")

## Transformation tree
![Transformation tree](/docs/frames.png "Transformation tree")

## Installation
To install packages needed for each robot. {$ROBOT_NAME} could be arlobot, padbot or wheelchair

### Download codes
```
cd ~/catkin_ws/src
git clone https://github.com/DTU-R3/DTU-R3-ROS.git
cd DTU-R3-ROS
git checkout origin/dtu-r3/{$ROBOT_NAME}
git submodule init
git submodule update
git submodule foreach git checkout origin/dtu-r3/{$ROBOT_NAME}
```

### Compile workspace
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Install dependencies
### Arlobot
```
sudo apt update
sudo apt upgrade
sudo apt install python-serial ros-kinetic-cv-bridge ros-kinetic-move-base-msgs ros-kinetic-nodelet ros-kinetic-robot-state-publisher ros-kinetic-tf ros-kinetic-xacro ros-kinetic-yocs-cmd-vel-mux ros-kinetic-yocs-velocity-smoother
```

### Raspiberry Pi camera
```
sudo apt install ros-kinetic-compressed-*
```

### mqtt
```
sudo apt install ros-kinetic-mqtt-bridge
sudo pip install inject paho-mqtt msgpack-python
```

### waypoint_nav
```
sudo apt-get install python-pip
sudo pip install pyproj
```

### padbot & wheelchair-jetson
```
sudo apt install ros-kinetic-rosserial*
```

## RUN codes
```
roslaunch arlobot_bringup arlobot.launch		# Run arlobot
roslaunch arlobot_bringup arlobot_laser.launch		# Run arlobot with RPLidar
roslaunch padbot padbot_u1.launch			# Run padbot
roslaunch wheelchair-jetson wheelchair_jetson.launch	# Run wheelchair
```

## Use two serial port device on Raspberry Pi
The Raspberry Pi has two serial ports on board, however, one is used by bluetooth by default. So in order to use two serial port device at the same time, we need to change the functionality of that port.


```
sudo raspi-config		# Enable serial port
sudo apt-get update		# Update the system
sudo apt-get upgrade
sudo nano /boot/config.txt	# Add device tree
```

In /boot/config.txt, we can disable the bluetooth by
```
dtoverlay=pi3-disable-bt
```
Or change the port functionality to miniusart if we want to have two device at the same time
```
dtoverlay=pi3-miniuart-bt
```

## Automatically run code when the machine boots, using system service (systemd)
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

# Delivery scenario
The demo is designed to ask the robot fetch the object and deliver it back to the user. Several technologies are involved in this demo such as voice/vision recognition, robot control, localisation, navigation and speech synthesis. The vision and voice recognition are done by the Google AIY Vision and Voice, respectively.

## Hardware list
* [Arlobot Kit](https://learn.parallax.com/tutorials/arlo)
* [Google AIY Vision Kit](https://aiyprojects.withgoogle.com/vision)
* [Google AIY Voice Kit](https://aiyprojects.withgoogle.com/voice)
* [Raspberry Pi](https://www.raspberrypi.org/)
* [Raspberry Pi camera](https://www.raspberrypi.org/products/camera-module-v2/)
* [RPLidar](http://www.slamtec.com/en/Lidar/A3)
* Speaker

## Customise scenario
The scenario consists of a number of tasks that can be custimised to adapt new scenario. The tasks is sent to ROS as json. An example of tasks can be found [here](https://github.com/DTU-R3/DTU-R3-ROS/blob/master/waypoint_nav/src/tasks.json). The tasks are:

* Waypoint: Ask the robot to run through a series of waypoints.
* Waypoint_fid: Ask the robot to run through a series of waypoints, stop the task when target fiducial is observed.
* Corridor_fid: Make the robot run in [corridor mode](https://github.com/DTU-R3/DTU-R3-ROS/blob/master/waypoint_nav), stop the task when target fiducial is observed.
* Speak: Ask the robot to speak something.
* Speak_cmd: Ask the robot to keep speaking, stop the task when target command is received.

## Deployment
In order to quickly reproduce the demo without too much professional knowledge, it can also be deployed on Raspberry with default Raspian image through docker.

# License
DTU-R3-ROS is licensed under the **BSD 3-clause "New" or "Revised"** License - see the [LICENSE.md](LICENSE) file for details

