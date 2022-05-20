# UR5-Robot-Simulation
The following project focuses on simulating a UR5 robot on Gazebo.

## Table of contents
* [General info](#general-info)
* [Setup](#setup)
* [Implementation](#implementation)

## General info
The project comprises of two tasks:  
* The first task focuses on publishing joint angles as a function of sine waves on the joints of ur5 robot in Gazebo.  
* The second task focuses on Pseudo-Admittance Controller for our simulation. The idea was to create a node that generates a force in X, Y or Z direction of the end-effector and calculates target joint angles based on a target cartesian pose using an open source inverse kinematic solver, called KDL. These joint angles are then pushed backed on the ur5 robot simulation on Gazebo. Some of the code in the second task is referenced from: https://github.com/dairal/ur5-tcp-position-control

## Setup
The project requires Ubuntu 16.04 and ROS Kinetic (Full desktop version is recommended). In order to get the universal robot packages, use:
```
sudo apt-get install ros-kinetic-universal-robot
```
Once you have downloaded ROS kinetic (including the universal robot package) and initialized catkin workspace, clone this repository in the src folder of the catkin workspace and build the code by: 
```
catkin_make
```
Make sure that you source the files:
```
source devel/setup.bash
```

## Implementation
<b>NOTE: I have added a small 20 second delay in order for the simulation environment to load and then start the publishing of the joint angles. You can remove the delay or increase it inside the launch files located in joint_pub/launch. </b>  

To run the first task:
```
roslaunch joint_pub task1.launch
```
In the first task, the simulation is going to run for 3 seconds and then the node will shutdown (except the gazebo node). The cpp file for this task is located <a href="https://github.com/sameeranees/UR5-Robot-Simulation/blob/main/joint_pub/src/joint_publisher.cpp)">here</a>.    
To run the second task:
```
roslaunch joint_pub task1.launch
```
In the second task , the simulation time and the force applied on the end-effector are given in the launch file, which you can change according to your implementation. Note that applying forces that are considered non optimal might make the robot crash into itself. The cpp file for this task is located <a href="https://github.com/sameeranees/UR5-Robot-Simulation/blob/main/joint_pub/src/ik_solver_2.cpp)">here</a>.

