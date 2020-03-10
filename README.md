# iRoPro - interactive Robot Programming tool
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__rapid_pbd__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__rapid_pbd__ubuntu_trusty_amd64__binary/)

iRoPro is an interactive Robot Programming tool, currently developed for the Baxter robot.
The goal of the system is to provide an easy way to program low- and high-level manipulation actions that can be used with a task planner to complete more complex tasks.
It is based on [Rapid PbD](https://github.com/jstnhuang/rapid_pbd/) which is a programming by demonstration (PbD) system for the PR2, Fetch, and Baxter robots.

## Program model
Users use the iRoPro interface to create *actions* and *problems*.
A video of the working system can be seen [here](https://youtu.be/NgaTPG8dZwg)

# Clone rapid_pbd
Clone this repository and its messages:
```
cd ~/catkin_ws/src
git clone git@github.com:ysl208/rapid_pbd.git
git clone git@github.com:ysl208/rapid_pbd_msgs.git
```

## What is needed

- As it is based on Rapid PbD, the following instructions are taken from [Rapid PbD set up](https://github.com/jstnhuang/rapid_pbd/wiki/Rapid-PbD-development-setup):

# Build the backend
## Backend dependencies

**MongoDB**

Install `pymongo` version 3.4:
```
$ sudo pip install pymongo==3.4
$ python -c 'import pymongo; print pymongo.version'
3.4.0
```

[This fork](https://github.com/jstnhuang/mongodb_store) of `mongodb_store` is necessary to avoid startup errors (see [strands-project/mongodb_store#196](https://github.com/strands-project/mongodb_store/issues/196)):
```
cd ~/catkin_ws/src
git clone git@github.com:jstnhuang/mongodb_store.git
```
Or just install the official version:
```
sudo apt-get install ros-melodic-mongodb-store
```

# Transform_graph
The Rapid PbD part uses a transform_graph for object perception:
```
cd ~/catkin_ws/src
git clone git@github.com/jstnhuang/surface_perception.git
git clone git@github.com/jstnhuang/transform_graph.git
```

# MoveIt for motion planning
Install [MoveIt](https://moveit.ros.org/install/) which is being used for motion planning for Baxter:
```
sudo apt-get install ros-<distribution>-moveit
sudo apt-get install ros-melodic-simple-grasping

```

**rosdep**

Get other dependencies through `rosdep`:
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=<distribution> -y
```
For ROS melodic, you might need these:
```
git clone https://github.com/RobotWebTools/ros_web_video.git
git clone https://github.com/jstnhuang/robot_markers.git
git clone https://github.com/jstnhuang/moveit_goal_builder.git
```

## Build it
You might want to install `python-catkin-tools` first:
```
sudo apt-get install python-catkin-tools
```
To build it, run
```
catkin build rapid_pbd
```

# Frontend
## Build the frontend
**One-time setup**

First, [install Node.js](https://github.com/hcrlab/wiki/wiki/Resources%3A-Web-Development%3A-Installing-Node).

Then, install Polymer and bower:
```
npm install -g bower polymer-cli
```

Go to the `frontend` folder and run `bower update`:
```
cd rapid_pbd/frontend
bower update
```

**Development and deployment**

When you are developing, just go to the frontend folder and run `polymer serve`:
```
cd frontend
polymer serve
```

Polymer will tell you the address of the app (usually localhost:8081).


## Collada file server
In order to see robot meshes in the web interface, you will need to run a Collada file server with cross-origin resource sharing on.

Follow the instructions in [Serving URDFs](https://github.com/hcrlab/wiki/wiki/Resources%3A-Web-Development%3A-Serving-URDF), ignoring the "In production" section.

# Mock point clouds
When running Rapid PbD in simulation, the system expects point clouds to be published to the `/mock_point_cloud` topic.
This allows you to try out different perception scenarios without having to change the Gazebo world.

You will need to write your own code to save point clouds in the `base_link` frame and publish them to `/mock_point_cloud`.
Or, you can use the [CSE 481C](https://github.com/cse481sp17/cse481c) class code:
```
cd ~/catkin_ws/src
git clone git@github.com:cse481sp17/cse481c.git
catkin build
rosrun perception save_cloud NAME.bag
rosrun applications publish_saved_cloud NAME.bag
```

- Kinect xBox/Kinect2 installed and calibrated (or another rgbd depth camera)
- For Baxter eyes, install the package [baxter_eyes](https://github.com/Anne-Gaisne/baxter_eyes)
- You will further require [PDDL planner](http://docs.ros.org/indigo/api/pddl_planner/html/) developed by Ryohei Ueda:
```
sudo apt-get install ros-indigo-pddl-planner ros-indigo-pddl-planner-msgs
(optional) sudo apt-get install ros-indigo-pddl-planner-viewer
```

## Commands to run
Run the following commands in a separate tab
```
roscore
```
### (Optional) simulation in Gazebo
Start the robot simulation before you run the following commands.
Follow [these](https://sdk.rethinkrobotics.com/wiki/Simulator_Installation) instructions for the Baxter robot.

### enable baxter robot
`rosrun baxter_tools enable_robot.py -e && rosrun baxter_tools tuck_arms -u`

### disable baxter robot
`rosrun baxter_tools enable_robot.py -d && rosrun baxter_tools tuck_arms -t`

### start baxter gripper and joint action servers & gripper cuff buttons
`rosrun baxter_interface gripper_action_server.py & rosrun baxter_interface joint_trajectory_action_server.py -l both & rosrun baxter_interface head_action_server.py & rosrun baxter_examples gripper_cuff_control.py` 

### iRoPro nodes (all in separate tabs)
```
roslaunch rapid_pbd baxter_moveit.launch right_electric_gripper:=false left_electric_gripper:=true
roslaunch rapid_pbd web_prereqs.launch
roslaunch rapid_pbd baxter.launch sim:=false kinect:=true --screen
roslaunch rapid_pbd editor.launch robot:=baxter --screen
```

### PDDL planner
This allows iRoPro to send rostopics to a [PDDL planner](http://docs.ros.org/indigo/api/pddl_planner/)
`roslaunch pddl_planner pddl_ff.launch`

### kinect xbox 
`roslaunch freenect_launch freenect.launch`

### start local web app - on http://localhost:9080
```
cd ~/catkin_ws/src/rapid_pbd/frontend
static-server
```

## Optional commands
### Baxter eyes
`roslaunch baxter_eyes baxter_eyes.launch`

### kinect2
`roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true`

### start robot model for visualization (this might crash the browser)
```
cd ~/local/collada
caddy
```

### instead of static server (from rapid_pbd) start web app - on http://localhost:8081
```
cd ~/catkin_ws/src/rapid_pbd/frontend
polymer-serve
```

# Bugs & Troubleshooting
Please also search your problem under 'Issues'. Feel free to create a new issue to keep track of solutions.

## The executed poses do not match the demonstrated ones
This is because the calibration of the kinect to the robot is not very accurate. Try to recalibrate it.
Last time we used this command to publish the transformation, which you can adjust.

`rosrun tf static_transform_publisher 0.189 0.03 0.775 -0.014 0.876 -0.014 base camera_link 100`

## The interface doesn't display the detected objects
Click on the Refresh button, if it still doesn't show then open the side menu on the visualisation and uncheck & check 'Surface segmentation'

## the arms don't move anymore, even if I just try to tuck them
It's good to restart the robot by runing disable and untuck arms
`rosrun baxter_tools enable_robot.py -d && rosrun baxter_tools tuck_arms -u`

# Citation
```
@inproceedings{Liang2019a,
	author = {Ying Siu Liang and Damien Pellier and Humbert Fiorino and Sylvie Pesty},
	title = {End-User Programming of Low- and High-Level Actions for Robotic Task Planning},
	booktitle = {IEEE International Conference on Robot and Human Interactive Communication (Ro-MAN)},
	address = {New Delhi, India},
	month = {October},
	year = {2019}
}
```
