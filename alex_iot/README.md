# ROS Package for IoT platform

## Features
* Use mqtt_bridge to remap mqtt commands to ROS topics
* Use espeak speech synthesizer to make the robot talk

## Prerequisites
ROS Kinetic
Mqtt bridge

## Installation
To install packages needed for each robot. $ROBOT_NAME could be arlobot, padbot or wheelchair

### Download codes
```
cd ~/catkin_ws/src
git clone https://github.com/DTU-R3/DTU-R3-ROS.git
cd DTU-R3-ROS
git checkout origin/dtu-r3/$ROBOT_NAME
git submodule init
git submodule update
git submodule foreach git checkout origin/dtu-r3/$ROBOT_NAME
```

### mqtt
```
sudo apt install ros-kinetic-mqtt-bridge
sudo pip install inject paho-mqtt msgpack-python
```

## RUN codes
```
roslaunch alex_iot alex_iot.launch		# Run mqtt
roslaunch alex_iot espeak.launch		# Run espeak
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

## License
DTU-R3-ROS is licensed under the **BSD 3-clause "New" or "Revised"** License - see the [LICENSE.md](LICENSE) file for details

