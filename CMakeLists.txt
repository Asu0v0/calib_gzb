cmake_minimum_required(VERSION 3.0.2)
project(calib_gzb)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
)

include_directories(
)
install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(move_ball_plugin SHARED src/move_ball_plugin.cpp)
target_link_libraries(move_ball_plugin ${GAZEBO_LIBRARIES})

install(TARGETS move_ball_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)