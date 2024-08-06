# Unitree_GO1_Dan
An overview of my Unitree GO1 WS 

Note that this workspace is a workspace which is derived from the original unitree github which did not work so has been edited.

I have made edits to the unitree_ros_to_real package which unitree provide. Key files include:
unitree_legged_real->src->exe:
dan_teleop_basic.cpp //FINISHED
dan_teleop_advanced.cpp //FINISHED


**Dependencies**
Unitree_ros - See Unitree Github
Unitree_legged_sdk - See Unitree Github
Real-time-ROS-EEG-decoding-test-using-turtlesim (See DanielM1tchell Github)

I also started to use the RealSense Camera too so I have included dependencies for that too. Its alot of github packages...


elevation_mapping- https://github.com/ANYbotics/elevation_mapping
geometry2- https://github.com/ros/geometry2
grid_map - https://github.com/ANYbotics/grid_map.git
kindr - https://github.com/ANYbotics/kindr.git
kindr_ros - https://github.com/ANYbotics/kindr_ros.git
lcm - https://github.com/lcm-proj/lcm.git
librealsense - https://github.com/IntelRealSense/librealsense.git
message_logger - https://github.com/ANYbotics/message_logger.git
navigation - https://github.com/ros-planning/navigation.git
octomap - https://github.com/OctoMap/octomap.git
pcl - https://github.com/PointCloudLibrary/pcl.git // THIS IS A TOUGH ONE TO DOWNLOAD 
realsense_ros - https://github.com/IntelRealSense/realsense-ros.git
http://wiki.ros.org/teleop_twist_keyboard // Unused in the end but good to have
turtlebot3 - https://github.com/ROBOTIS-GIT/turtlebot3.git //Needed when testing simulation first for elevation mapping



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





