# Mqtt bridge between GamesOnTrack and ROS

The package is used to publish GamesOnTrack sensor data to ROS

## Prerequisites
ROS Kinetic

## Installation
```
sudo apt install ros-kinetic-mqtt-bridge
sudo pip install inject paho-mqtt msgpack-python
```

## How to use
* Change the message type of ros topic wanted.
* In /src/r3-got_params.yaml, change the 'topic_from' and 'topic_to' as mqtt topic and ros topic respectively.
* Run launch file

## Run the code
```
roslaunch mqtt r3-got.launch
```
