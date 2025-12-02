cmake_minimum_required(VERSION 3.10)
project(sdf_contact_estimation_demo)

find_package(ament_cmake REQUIRED)

# ##############################################################################
# Install ##
# ##############################################################################

install(DIRECTORY launch rviz urdf config DESTINATION share/${PROJECT_NAME})

# ##############################################################################
# Package ##
# ##############################################################################

ament_package()
