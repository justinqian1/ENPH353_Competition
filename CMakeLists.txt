cmake_minimum_required(VERSION 2.8.3)

project(team5_code)

find_package(catkin REQUIRED rospy std_msgs geometry_msgs sensor_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/line_follow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)