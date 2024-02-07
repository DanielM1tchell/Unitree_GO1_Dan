cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

set(CMAKE_CXX_FLAGS "-O3 -fPIC -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    unitree_legged_msgs
    unitree_legged_sdk
)

catkin_package()

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

set(EXTRA_LIBS -pthread ${unitree_legged_sdk_LIBRARIES})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${unitree_legged_sdk_INCLUDE_DIRS}
    ~/unitree_go1_ws/src/unitree_legged_sdk/include/
)


add_executable(ros_example_walk src/exe/example_walk.cpp)
target_link_libraries(ros_example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dan_walk src/exe/dan_walk.cpp)
target_link_libraries(dan_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dan_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dan_walk_v0_2 src/exe/dan_walk_v0_2.cpp)
target_link_libraries(dan_walk_v0_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dan_walk_v0_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dan_walk_v0_4 src/exe/dan_walk_v0_4.cpp)
target_link_libraries(dan_walk_v0_4 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dan_walk_v0_4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dan_walk_v0_6 src/exe/dan_walk_v0_6.cpp)
target_link_libraries(dan_walk_v0_6 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dan_walk_v0_6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dan_walk_v0_8 src/exe/dan_walk_v0_8.cpp)
target_link_libraries(dan_walk_v0_8 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dan_walk_v0_8 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dan_walk_v1_0 src/exe/dan_walk_v1_0.cpp)
target_link_libraries(dan_walk_v1_0 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dan_walk_v1_0 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dan_teleop_basic src/exe/dan_teleop_basic.cpp)
target_link_libraries(dan_teleop_basic ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dan_teleop_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dan_teleop_advanced src/exe/dan_teleop_advanced.cpp)
target_link_libraries(dan_teleop_advanced ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(dan_teleop_advanced ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_example_position src/exe/example_position.cpp)
target_link_libraries(ros_example_position ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_example_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(state_sub src/exe/state_sub.cpp)
target_link_libraries(state_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(state_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_udp src/exe/ros_udp.cpp)
target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(control_via_keyboard src/exe/control_via_keyboard.cpp)
target_link_libraries(control_via_keyboard ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(control_via_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(twist_sub src/exe/twist_sub.cpp)
target_link_libraries(twist_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(twist_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

