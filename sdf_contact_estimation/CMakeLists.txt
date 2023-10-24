cmake_minimum_required(VERSION 3.0.2)
project(sdf_contact_estimation)

#set(CMAKE_BUILD_TYPE "Debug")

# Options
set(BUILD_BENCHMARKS ON)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  voxblox
  voxblox_ros
  pcl_ros
  geometry_msgs
  visualization_msgs
  voxblox_msgs
  moveit_core
  moveit_msgs
  hector_math
  hector_pose_prediction_interface
  hector_stability_metrics
  hector_pose_prediction_ros
  eigen_conversions
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${CERES_INCLUDE_DIRS}
  LIBRARIES sdf_contact_estimation
  CATKIN_DEPENDS roscpp sensor_msgs voxblox voxblox_ros pcl_ros geometry_msgs visualization_msgs voxblox_msgs moveit_core moveit_msgs hector_math hector_pose_prediction_interface hector_stability_metrics hector_pose_prediction_ros eigen_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(HEADERS
  include/${PROJECT_NAME}/sdf_contact_estimation.h
  include/${PROJECT_NAME}/optimization/pose_optimizer.h
  include/${PROJECT_NAME}/robot_model/shape_model.h
  include/${PROJECT_NAME}/robot_model/basic_shapes/shape_base.h
  include/${PROJECT_NAME}/robot_model/basic_shapes/rectangle_shape.h
  include/${PROJECT_NAME}/robot_model/basic_shapes/cylinder_shape.h
  include/${PROJECT_NAME}/sdf/interpolated_voxblox_tsdf.h
  include/${PROJECT_NAME}/sdf/interpolated_voxblox_esdf.h
  include/${PROJECT_NAME}/sdf/sdf_model.h
  include/${PROJECT_NAME}/test_scenarios.h
  include/${PROJECT_NAME}/visualisation.h
  include/${PROJECT_NAME}/util/utils.h
  include/${PROJECT_NAME}/util/timing.h
  include/${PROJECT_NAME}/optimization/falling_pose_optimizer.h
  include/${PROJECT_NAME}/optimization/rotation_pose_optimizer.h
  include/${PROJECT_NAME}/robot_model/shape_collision_types.h
)
set(SOURCES
  src/sdf_contact_estimation.cpp
  src/optimization/pose_optimizer.cpp
  src/sdf_model.cpp
  src/test_scenarios.cpp
  src/visualisation.cpp
  src/util/utils.cpp
  src/robot_model/shape_model.cpp
  src/robot_model/basic_shapes/shape_base.cpp
  src/optimization/falling_pose_optimizer.cpp
  src/optimization/rotation_pose_optimizer.cpp
  src/robot_model/shape_collision_types.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/sdf_contact_estimation_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

##################
## Benchmarking ##
##################
find_package(benchmark)
if (BUILD_BENCHMARKS AND ${benchmark_FOUND})
  add_executable(pose_estimation_benchmark benchmark/pose_estimation_benchmark.cpp)
#  target_compile_options(benchmark_obstacle_distance_map PRIVATE -march=native -mtune=native -O3)
  target_link_libraries(pose_estimation_benchmark ${PROJECT_NAME} benchmark benchmark_main)

  install(TARGETS pose_estimation_benchmark
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

#############
## Install ##
#############
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
