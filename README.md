# RESEARCH TRACK 2 - Assignment 1: ros2 branch
The purpose of this package is to communicate with nodes written in ROS1 package to control a non-holonomic mobile robot in a Gazebo environment. The user can choose to start the robot and to stop it. In the moment when the user decides to make moving the robot, a random pose is choosen and the robot start moving in that direction. This movement continues until the robot reachs the goal and a new random pose is sent. 
## Description of the branch
In this folder you can find: 
- launch: here it is contained the *sim_launch.py*, needed to instantiate a container and to load the components into. 
- src: contains the two nodes implemented as C++ files
  - *position_server*: implements the server for a random (x,y,theta) pose.
  - *state_machine*: calls the random goal and sends to *go_to_point* service, when the user wants to move the robot.
## How to compile and run the package 
In order to communicate with ROS1, we need three different steps: 
- Launch the component from ROS!, so in the shell where you sourced ROS1

```
roslaunch rt2_assignment1 bridge_sim.launch
```
- Run the ROS1 bridge, where you source both ROS1 and ROS2
```
ros2 run ros1_bridge dynamic_bridge
```
- Launch the container with the componentes implemented, where you sourced ROS2
```
roslaunch rt2_assignment1 sim_launch.py
```
## Documentation
In the doc folder, beside this README, it is possible to find all the documentation generated with Doxygen 
