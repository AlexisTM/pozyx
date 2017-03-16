cmake_minimum_required(VERSION 2.8.3)
project(pozyx)
find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pozyx
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)

catkin_install_python(PROGRAMS
  ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)
