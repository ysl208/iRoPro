# Rapid PbD
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__rapid_pbd__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__rapid_pbd__ubuntu_trusty_amd64__binary/)

Rapid PbD is a programming by demonstration (PbD) system for the PR2, Fetch, and Baxter robots.
The goal of the system is to provide an easy way to program manipulation actions that can be used in other applications.

## Program model
Users use the Rapid PbD interface to create *programs*.
A program is represented using the `rapid_pbd_msgs/Program` msg.
The system provides an actionlib interface for running programs.

A program consists of a sequence of *steps*, and each step consists of one or more *actions*.
There can be different types of actions, including moving the arm, moving the head, and detecting tabletop objects.
The actions of a step are run in parallel, but the steps run in sequence.
For example, in one step, you can point the head down and move the robot's arms to the side, and in the next step, you can detect tabletop objects.

## Getting started
- [Development setup](https://github.com/jstnhuang/rapid/wiki/Rapid-PbD-development-setup)
- [Running Rapid PbD](https://github.com/jstnhuang/rapid/wiki/Running-Rapid-PbD)

## Commands to run that are different (for the real robot)
roscore

rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface gripper_action_server.py &
rosrun baxter_interface joint_trajectory_action_server.py -l both &
rosrun baxter_interface head_action_server.py

roslaunch rapid_pbd baxter_moveit.launch right_electric_gripper:=false left_electric_gripper:=true
roslaunch rapid_pbd web_prereqs.launch
roslaunch rapid_pbd baxter.launch sim:=false kinect:=true --screen
roslaunch rapid_pbd editor.launch robot:=baxter --screen

# kinect + PDDL planner
roslaunch pddl_planner_viewer pddl_planner_sample-pddl.launch
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
roslaunch freenect_launch freenect.launch

cd ~/local/collada
caddy

cd ~/catkin_ws/src/rapid_pbd/frontend
static-server

Go to http://localhost:9080
