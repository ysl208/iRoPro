# iRoPro - interactive Robot Programming tool
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__rapid_pbd__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__rapid_pbd__ubuntu_trusty_amd64__binary/)

iRoPro is an interactive Robot Programming tool, currently developed for the Baxter robot.
It uses [Rapid PbD](https://github.com/jstnhuang/rapid_pbd/) which is a programming by demonstration (PbD) system for the PR2, Fetch, and Baxter robots.
The goal of the system is to provide an easy way to program low- and high-level manipulation actions that can be used with a task planner to complete more complex tasks.

## Program model
Users use the iRoPro interface to create *programs*.
A program is represented using the `iRoPro_msgs/Program` msg.
The system provides an actionlib interface for running programs.

A program consists of a sequence of *steps*, and each step consists of one or more *actions*.
There can be different types of actions, including moving the arm, moving the head, and detecting tabletop objects.
The actions of a step are run in parallel, but the steps run in sequence.
For example, in one step, you can point the head down and move the robot's arms to the side, and in the next step, you can detect tabletop objects.

## Getting started
- [Development setup](https://github.com/jstnhuang/rapid/wiki/Rapid-PbD-development-setup)

## Commands to run (for the real robot)
`roscore`

### enable baxter robot
`rosrun baxter_tools enable_robot.py -e && rosrun baxter_tools tuck_arms -u`

### disable baxter robot
`rosrun baxter_tools enable_robot.py -d && rosrun baxter_tools tuck_arms -t`

### start baxter gripper and joint action servers & gripper cuff buttons
`rosrun baxter_interface gripper_action_server.py & rosrun baxter_interface joint_trajectory_action_server.py -l both & rosrun baxter_interface head_action_server.py & rosrun baxter_examples gripper_cuff_control.py` 

### iRoPro nodes (all in separate tabs)
`roslaunch rapid_pbd baxter_moveit.launch right_electric_gripper:=false left_electric_gripper:=true`

`roslaunch rapid_pbd web_prereqs.launch`

`roslaunch rapid_pbd baxter.launch sim:=false kinect:=true --screen`

`roslaunch rapid_pbd editor.launch robot:=baxter --screen`


### PDDL planner
##  This allows iRoPro to send rostopics to a [PDDL planner](http://docs.ros.org/indigo/api/pddl_planner/)
`roslaunch pddl_planner pddl_ff.launch`

### kinect xbox 
`roslaunch freenect_launch freenect.launch`

### start local web app - on http://localhost:9080
`cd ~/catkin_ws/src/rapid_pbd/frontend`

`static-server`

### Optional commands
### Optional Baxter eyes
`roslaunch baxter_eyes baxter_eyes.launch`

### kinect2
`roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true`

### start robot model for visualization (this might crash the browser)
`cd ~/local/collada`
`caddy`

### instead of static server (from rapid_pbd) start web app - on http://localhost:8081
cd ~/catkin_ws/src/rapid_pbd/frontend
polymer-serve

### Bugs & Troubleshooting
Please search your problem under 'Issues' first. Feel free to create a new issue to keep track of solutions.

### The executed poses do not match the demonstrated ones
This is because the calibration of the kinect to the robot is not very accurate. Try to recalibrate it.
Last time we used this command to publish the transformation, which you can adjust.

`rosrun tf static_transform_publisher 0.189 0.03 0.775 -0.014 0.876 -0.014 base camera_link 100`

### The interface doesn't display the detected objects
Click on the Refresh button, if it still doesn't show then open the side menu on the visualisation and uncheck & check 'Surface segmentation'

### the arms don't move anymore, even if I just try to tuck them
It's good to restart the robot by runing disable and untuck arms
`rosrun baxter_tools enable_robot.py -d && rosrun baxter_tools tuck_arms -u`

# Citation
``
@inproceedings{Liang2019a,
	author = {Ying Siu Liang and Damien Pellier and Humbert Fiorino and Sylvie Pesty},
	title = {End-User Programming of Low- and High-Level Actions for Robotic Task Planning},
	booktitle = {IEEE International Conference on Robot and Human Interactive Communication (Ro-MAN)},
	address = {New Delhi, India},
	month = {October},
	year = {2019}
}``
