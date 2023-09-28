cmake_minimum_required(VERSION 3.0.2)
project(sdf_contact_estimation_demo)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY
  launch
  rviz
  urdf
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
