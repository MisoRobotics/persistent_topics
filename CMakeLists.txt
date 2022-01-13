cmake_minimum_required(VERSION 3.0.2)
project(persistent_topics)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS rospy
)

catkin_install_python(
    PROGRAMS
        nodes/single_channel_${PROJECT_NAME}_node
        nodes/multi_channel_${PROJECT_NAME}_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION})
