#!/bin/bash
#placing the script in communication with the robot
#change ROS_MASTER_URI and ROS_IP to your environment
source /opt/ros/indigo/setup.bash
source ./devel/setup.bash
./baxter.sh &
export ROS_PACKAGE_PATH=~/git/byu/development:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://192.168.0.133:11311
export ROS_IP=192.168.0.132

if [ $1 = "-t" ] || [ $1 = "-u" ] #for tucking and untucking arm
then
	rosrun baxter_tools enable_robot.py -e
	rosrun baxter_tools tuck_arms.py "$1"
elif [ $1 = "-d" ] || [ $1 = "-e" ] || [ $1 = "-r" ]
then
	rosrun baxter_tools enable_robot.py "$1"
elif [ $1 = "-y" ] #to display eyes on baxter tablet
then 
	#kill other display node
	rosnode kill /baxter_camera_display_node
	rosrun baxter_eyes baxter_arm_mvmt_detector_node &
	rosrun baxter_eyes baxter_eyes_node ./src/baxter_eyes/src/assets/eyes-contour.png ./src/baxter_eyes/src/assets/pupils.png &
elif [ $1 = "-c" ] #to display cameras on baxter tablet
then
	#kill other display node
	rosnode kill /baxter_arm_mvmt_detector_node &
	rosnode kill /baxter_eyes_node &
	#close head camera (only two camera can be open simultaneously on baxter)
	rosrun baxter_tools camera_control.py -c head_camera
	if [ $2 = "/cameras/left_hand_camera/image" ] || [ $3 = "/cameras/left_hand_camera/image" ] || [ $4 = "/cameras/left_hand_camera/image" ] || [ $5 = "/cameras/left_hand_camera/image" ]
	then
		rosrun baxter_tools camera_control.py -o left_hand_camera
		if [ $2 = "/cameras/head_camera/image" ] || [ $3 = "/cameras/head_camera/image" ] || [ $4 = "/cameras/head_camera/image" ] || [ $5 = "/cameras/head_camera/image" ]
		then
			rosrun baxter_tools camera_control.py -c right_hand_camera
			rosrun baxter_tools camera_control.py -o head_camera
		fi
	fi

	if [ $2 = "/cameras/right_hand_camera/image" ] || [ $3 = "/cameras/right_hand_camera/image" ] || [ $4 = "/cameras/right_hand_camera/image" ] || [ $5 = "/cameras/right_hand_camera/image" ]
	then
		rosrun baxter_tools camera_control.py -o right_hand_camera
		if [ $2 = "/cameras/head_camera/image" ] || [ $3 = "/cameras/head_camera/image" ] || [ $4 = "/cameras/head_camera/image" ] || [ $5 = "/cameras/head_camera/image" ]
		then
			rosrun baxter_tools camera_control.py -c left_hand_camera
			rosrun baxter_tools camera_control.py -o head_camera
		fi
	fi
	rosrun baxter_eyes baxter_camera_display_node "$2" "$3" "$4" "$5" &
fi
