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
Once you have built the package, you can launch the nodes. These nodes utilize localization in a VICON environment and are designed to work with ROBOTIS's [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [VICON Bridge](https://github.com/ethz-asl/vicon_bridge) packages. Ensure that the vicon_bridge node is running on your remote Ubuntu machine.

Onboard the TurtleBot3, run the bringup node.

```roslaunch turtlebot3_bringup turtlebot3_robot.launch```

Then, initialize the waypoint navigation and logger.

```rosrun messop_ugv messop```
```rosrun messop_ugv logger```

To control the TurtleBot3, publish a MessToUGV message from the [mess_msgs package](https://github.com/marinarasauced/mess_msgs).

## Messop node information
Topics:
- ```/cmd_vel```: Publishes ```geometry_msgs/Twist``` control input to bringup node.
- ```/messop/logger/coefficients```: Publishes ```mess_msgs/CalibrateUGV``` coefficients that calibrate Odometry and SLAM to VICON environment.
- ```/messop/logger/global```: Publishes ```mess_msgs/StateUGV``` estimated from VICON, Odometry, and SLAM.
- ```/messop/logger/odom```: Publishes ```mess_msgs/StateUGV``` from Odometry and SLAM calibrated to the VICON environment.
- ```/messop/messop/vertex```: Subscribes to ```mess_msgs/MessToUGV``` vertex and operation type.
- ```/messop/messop/interference```: Subscribes to ```std_msgs/Bool``` for deliberately ignoring VICON localization.
- ```/vicon/{UGV_NAME}/{UGV_NAME}```: Subscribes to ```geometry_msgs/TransformStamped```VICON localization.

## Logger node information
Topics:
- ```/messop/logger/flag```: Subscribes to ```mess_msgs/MessToUGVLogger``` indicator to log data as CSV files.
- ```/cmd_vel```: Subscribes to ```geometry_msgs/Twist``` control input from messop node.
- ```/messop/logger/coefficients```: Subscribes to ```mess_msgs/CalibrateUGV``` coefficients that calibrate Odometry and SLAM to VICON environment.
- ```/messop/logger/global```: Subscribes to ```mess_msgs/StateUGV``` estimated from VICON, Odometry, and SLAM.
- ```/messop/logger/odom```: Subscribes to ```mess_msgs/StateUGV``` from Odometry and SLAM calibrated to the VICON environment.
- ```/vicon/{UGV_NAME}/{UGV_NAME}```: Subscribes to ```geometry_msgs/TransformStamped``` VICON localization.