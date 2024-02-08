# messop_ugv
ROS Noetic nodes for TurtleBot3s. Allows for time-optimal waypoint navigation using primarilly VICON localization with onboard Odometry and SLAM during occlusion periods or deliberate interference.






## Installation requirements
- A TurtleBot3 device - *if you are working on a new device, following [these steps](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to install ROS Noetic*
- An Ubuntu 20.04 machine
- A network with internet

## Installation instructions
To build from source, go to your catkin_ws ```cd ~/catkin_ws/src```.

Download the source for these nodes by running

```git clone https://github.com/marinarasauced/messop_ugv.git```

Compilete the code with ```catkin_make```.

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
