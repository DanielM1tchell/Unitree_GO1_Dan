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

**A- Basic Teleoperation**<br>
Use WASD controls via keyboard
Terminal Window 1<br>
`roslaunch unitree_legged_real real.launch ctrl_level:=highlevel`

Terimnal Window 2<br>
`rosrun unitree_legged_real dan_teleop_basic`

**B- Advanced Teleoperation**<br>
Use WASD & z,x for forward curve left and right controls via keyboard
Terminal Window 1<br>
`roslaunch unitree_legged_real real.launch ctrl_level:=highlevel`

Terimnal Window 2<br>
`rosrun unitree_legged_real dan_teleop_advanced`

**C- Advanced Teleoperation**<br>
Use WASD & z,x for forward curve left and right controls via keyboard
Terminal Window 1<br>
`roslaunch unitree_legged_real real.launch ctrl_level:=highlevel`

Terimnal Window 2<br>
`rosrun unitree_legged_real dan_teleop_advanced`

**D- BCI Teleoperation of Unitree GO1 using previously collected data (simulation)**<br>
Robot will move via simulated BCI data so no teleoperation control here
Terminal Window 1<br>
`roslaunch unitree_legged_real real.launch ctrl_level:=highlevel`

Terimnal Window 2<br>
`rosrun unitree_legged_real dan_teleop_basic_BCI_Dan_Sim`

Terminal Window 3<br>
cd to the EEG folder- Still to replace this as I need to upload the file together
`python 3 EEG file`

**E- BCI Teleoperation of Unitree GO1 using LIVE Data**<br>
Robot will move via real BCI data
Terminal Window 1<br>
`roslaunch unitree_legged_real real.launch ctrl_level:=highlevel`

Terimnal Window 2<br>
`rosrun unitree_legged_real dan_teleop_basic_BCI_Dan_real`

Terminal Window 3<br>
cd to the EEG folder- Still to replace this as I need to upload the file together
`python 3 EEG file`

**F- IN PROGRESS- BCI Teleoperation of Unitree GO1 using LIVE Data and advanced controller which features motion planning**<br>
Robot will move via real BCI data and use motion planning to change modes which use 1 for walk, 2 for crawl and 3 for stair mode
Terminal Window 1<br>
`roslaunch unitree_legged_real real.launch ctrl_level:=highlevel`

Terimnal Window 2<br>
`rosrun unitree_legged_real dan_teleop_advanced_BCI_Dan_real`





