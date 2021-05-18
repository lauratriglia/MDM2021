# RESEARCH TRACK2 - Assignment 1: action branch
The purpose of this package is to control a non-holonomic mobile robot in a Gazebo environment. The user can choose to make the robot start moving or stop it. When the user gives the command to start moving, a random pose is chosen and the robot start moving. The user has the possibility to stop the robot, even if a random pose has been already chosen. When the user gives the command to stop the robot, the robot immediately stops and waits for another random pose.
## Description of the branch. 
In this folder you can find:
- action: contains the .action file *Pose.action*, needed to modify go_to_point as an action server.
- launch: contains the launch file *sim.launch*, that starts the simulation in a Gazebo environment.
- scripts: contains two nodes implemented as python scripts that set the behaviour of the robot 
  - *go_to_point*: this is the action server that manage the robot speed control, depending on the goal received.
  - *user_interface*: sends the request to start/stop the go_to_point behaviour asking to the user what the robot needs to do. 
- src: contains two nodes implemented as cpp file that set the behaviour of the robot
  - *position_server*: it is the server that generates a random position.
  - *state_machine*: manages the request of a new goal and it send the request as a goal to go_to_point action server. 

## How to compile and run the package
After cloning the package, it is necessary to build the package in the path of your own workspace, with the command:
```
catkin_make
```
When the package is build successfully, you can run the .launch file to launch all the nodes and the Gazebo simulation:
```
roslaunch rt2_assignment1 sim.launch
```
## Documentation
In the doc folder, beside this ReadMe, it is possibile to find all the documentation generated with Doxygen.
