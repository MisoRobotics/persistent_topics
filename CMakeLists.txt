cmake_minimum_required(VERSION 2.8.3)
project(persistent_topics)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
   CATKIN_DEPENDS rospy
)

catkin_install_python(PROGRAMS nodes/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
