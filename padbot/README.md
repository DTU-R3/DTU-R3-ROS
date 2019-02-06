# Propeller code for Padbot
The package publishes padbot odometry based on encoders counts and integrates packages needed for padbot scenario into one launch file.

## Prerequisites
ROS Kinetic

## Installation
```
sudo apt install ros-kinetic-rosserial*
```

## Run the code

### Run propeller codes

```
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun padbot padbot_u1.py
```

### Run the whole scenario
roslaunch padbot padbot_u1.launch 

### Arduino
Arduino for padbot can be found [here](https://github.com/DTU-R3/Wheelchairduino/tree/master/padbot)
