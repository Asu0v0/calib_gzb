cmake_minimum_required(VERSION 3.0.2)
project(calib_gzb)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_ball_plugin
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(move_ball_plugin SHARED src/move_ball_plugin.cpp)
target_link_libraries(move_ball_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS move_ball_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch worlds urdf models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
