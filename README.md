# messop_ugv
## Overview
This repo contains a ROS Noetic package for a TurtleBot3 that uses VICON feedback to navigate a flat lab environment. The package allows for three operations for time-optimal navigation: 1. a rotation 2. a rotation and then a translation 3. a rotation, a translation, and then a rotation. The target user for this repo is anyone with access to AVMI resources or a VICON system.
TurtleBot3 navigational package for MQP-RVC2401

## Installation requirements
- A TurtleBot3 device - *if you are working on a new device, following [these steps](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to install ROS Noetic*
- An Ubuntu 20.04 machine

## Installation instructions
From your remote Ubuntu device, log into the TurtleBot3 using SSH, and enter the device password.
```
ssh ubuntu@{IP_OF_TURTLEBOT3}
```
Navigate to the source folder of your catkin workspace and clone the repository. This step requires the TurtleBot3 to have an active internet connection.
```
cd ~/catkin_ws/src && git clone https://github.com/marinarasauced/turtlebot3_messop.git
```
Return to the catkin workspace and compile the package.
```
cd ~/catkin_ws && catkin_make
```
## Usage instructions
Once you have the package built, you can launch the node. The node relies on feedback from the VICON system and is designed to work with the [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [VICON bridge](https://github.com/ethz-asl/vicon_bridge) packages. Users publish a custom message type containing a point, a heading, and an operation integer, which the node interprets into control inputs to perform the desired operation.
  
Start a core on the desktop machine, then launch the bringup function in a terminal onboard the TurtleBot3's RasPi.
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
In a new terminal onboard the TurtleBot3's RasPi, run the node. 
```
rosrun turtlebot3_messop turtlebot3_messop
```
A message should display in the terminal indicating the node was successfully launched. The TurtleBot3 will perform a brief calibration to orient itself in the VICON environment, and will then wait for the user to publish a MESS2UGV message.
## Notes
- An active internet connection is required to install packages to the TurtleBot3 from GitHub repositories.
