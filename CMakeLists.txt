cmake_minimum_required(VERSION 2.8.3)
project(cmt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

SET("OpenCV_DIR" "/usr/local/share/OpenCV/")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  cv_bridge
  sensor_msgs
  dynamic_reconfigure
)

find_package(OpenCV 3.4.1 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
generate_dynamic_reconfigure_options(cfg/tuning.cfg)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cmt
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########


add_definitions ("-Wall")

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/cmt
)

add_executable (cmt_node src/common.cpp src/gui.cpp src/main.cpp
    src/CMT.cpp src/Consensus.cpp src/Fusion.cpp src/Matcher.cpp src/Tracker.cpp
    src/fastcluster/fastcluster.cpp)

target_link_libraries(cmt_node
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)


