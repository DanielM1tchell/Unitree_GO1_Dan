# Unitree_GO1_Dan
An overview of my Unitree GO1 WS 

Note that this workspace is a work in progress and features several changes to the standard unitree githb packages to ensure that the code actually works.
I have made edits to the unitree_ros_to_real package which unitree provide. Key files include:
unitree_legged_real->src->exe:
dan_teleop_basic.cpp //Finished
dan_teleop_advanced.cpp //Working progress



**Dependencies**
Unitree_ros
Unitree_legged_sdk 

**How to run code**

**A- Basic Teleoperation**
Terminal Window 1
`roslaunch unitree_legged_real real.launch ctrl_level:=highlevel`

Terimnal Window 2
`rosrun unitree_legged_real dan_teleop_basic`
