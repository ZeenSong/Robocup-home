*************************Note************************************
install darknet_ros, wit_ros and open_position packages before compiling code


*************************Run steps******************************* 
run command by fllowing command step by step

Step 1.change map in Tiago to our map

rosservice call /pal_map_manager/change_map "input: 'newluosifen" "

Step 2.launch core file of speech recognition

roslaunch wit_ros start.launch

Step 3.launch core file for pose estimation

roslaunch openpose_ros openpose_ros.launch

Step 4.launch darknet for object recognition 

roslaunch darknet_ros darknet_ros.launch

Step 5. start main programm for interaction

rosrun wit_ros poseget.py

