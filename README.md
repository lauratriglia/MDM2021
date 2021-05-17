# RESEARCH TRACK 2- Assignment 1: main branch
## Description of the branch

In the main branch you can find four nodes: two nodes implemented as python scripts and the other two as C++ file. Each folder contains:
- VRep_scene: to simulate the robot in a Coppelia environment, it is available a simple scene *my_scene.ttt* with a Pioneer p3dx mobile robot in an empty environment
- launch: there are thre different .launch file
   - sim.launch: starts the simulation in a Gazebo environment
   - bridge_sim.launch: it is dedicated to launch only the python scripts, due to the fact that the components of ros2 take care of the leaving part.
   - coppelia_sim.launch: starts the simulation in a Coppelia environment.
- scripts: two nodes implemented as python scripts that set the behaviour of the robot
   - *go_to_point*: this is the server that manage the robot speed control, depending on the goal received.
   - *user_interface*: sends the request to start/stop the go_to_point behaviour asking to the user what the robot needs to do.
- src: two nodes implemented as C++ file that set the behaviour of the robot
   - *position_server*: it is the server that generates a random position.
   - *state_machine*: manage the request of a new goal and it sends the request as a goal to go_to_point server.

## How to compile and run the package
After cloning the package, it is necessary to build the package in the path of your own workspace, with the command:

```
catkin_make
```
When the package is build successfully, you have three options, depending on what is your goal:
- to start the simulation in Gazebo, it is necessary to run:
```
roslaunch rt2_assignment1 sim.launch
```
- to start the simulation in Coppelia, it is necessary to run:
```
roslaunch rt2_assignment1 coppelia_sim.launch
```
 N.B: Unlike Gazebo, Coppelia needs to start separately.
- to launch only the python scripts, it is necessary to run:
```
roslaunch rt2_assignment1 bridge_sim.launch
```
