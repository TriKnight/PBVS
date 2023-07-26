cmake_minimum_required(VERSION 3.5)
project(dense_grasp)
set(CMAKE_CXX_STANDARD 14)
# %Tag(FULLTEXT)%

#System Dependencies
find_package(Eigen3 REQUIRED) 

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  sensor_msgs 
  roslib
  moveit_ros_planning_interface
)


include_directories(${EIGEN3_INCLUDE_DIR}) 
include_directories(${catkin_INCLUDE_DIRS})



## Declare a catkin package
catkin_package(
CATKIN_DEPENDS roscpp std_msgs sensor_msgs roslib moveit_ros_planning_interface
DEPENDS Open3D
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(move_group_tutorial src/move_group_tutorial.cpp)
target_link_libraries(move_group_tutorial ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
