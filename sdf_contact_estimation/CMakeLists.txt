cmake_minimum_required(VERSION 3.16)
project(sdf_contact_estimation)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_BENCHMARKS "Build pose estimation benchmark" ON)

# ##############################################################################
# Dependencies
# ##############################################################################

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(voxblox REQUIRED)
# voxblox_ros exports pcl_ros/pcl_conversions, which find PCL. PCLConfig finds
# VTK without its jsoncpp component, leaving VTK::jsoncpp referencing an
# undefined JsonCpp::JsonCpp target
find_package(jsoncpp CONFIG QUIET)
find_package(voxblox_ros REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(hector_math REQUIRED)
find_package(hector_pose_prediction_interface REQUIRED)
find_package(hector_stability_metrics REQUIRED)
find_package(hector_pose_prediction_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# system / non-ament libs
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)

# Ceres (optional)
find_package(Ceres QUIET)
if(Ceres_FOUND)
  message(STATUS "Found Ceres: ${Ceres_DIR}")
  set(CERES_INCLUDE_DIRS ${Ceres_INCLUDE_DIRS})
  set(CERES_LIBRARIES Ceres::ceres)
else()
  message(STATUS "Ceres not found. Building without explicit Ceres linkage.")
  set(CERES_INCLUDE_DIRS "")
  set(CERES_LIBRARIES "")
endif()

# Ament-style dependencies (ROS 2 / ament packages only)
set(THIS_PACKAGE_DEPENDENCIES
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    voxblox
    voxblox_ros
    voxblox_msgs
    moveit_core
    moveit_msgs
    hector_math
    hector_pose_prediction_interface
    hector_stability_metrics
    hector_pose_prediction_ros
    tf2_eigen
    # eigen_conversions
)

# ##############################################################################
# Build ##
# ##############################################################################

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
    include/${PROJECT_NAME}/robot_model/shape_collision_types.h)

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
    src/robot_model/shape_collision_types.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> ${CERES_INCLUDE_DIRS}
         ${voxblox_INCLUDE_DIRS} ${voxblox_ros_INCLUDE_DIRS})

# voxblox and voxblox_ros do not export CMake targets, use the ament variables
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp
         ${sensor_msgs_TARGETS}
         ${geometry_msgs_TARGETS}
         ${visualization_msgs_TARGETS}
         ${voxblox_LIBRARIES}
         ${voxblox_ros_LIBRARIES}
         ${voxblox_msgs_TARGETS}
         moveit_core::moveit_robot_model
         moveit_core::moveit_robot_state
         ${moveit_msgs_TARGETS}
         hector_math::hector_math
         hector_pose_prediction_interface::hector_pose_prediction_interface
         hector_stability_metrics::hector_stability_metrics
         hector_pose_prediction_ros::hector_pose_prediction_ros
         tf2_eigen::tf2_eigen
         ${CERES_LIBRARIES}
         yaml-cpp
         glog::glog)

# ##############################################################################
# Node    #
# ##############################################################################

add_executable(${PROJECT_NAME}_node src/sdf_contact_estimation_node.cpp)

target_include_directories(${PROJECT_NAME}_node
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# ##############################################################################
# Benchmarking ##
# ##############################################################################

find_package(benchmark QUIET)
if(BUILD_BENCHMARKS AND benchmark_FOUND)
  add_executable(pose_estimation_benchmark
                 benchmark/pose_estimation_benchmark.cpp)
  # target_compile_options(pose_estimation_benchmark PRIVATE -march=native
  # -mtune=native -O3)

  target_link_libraries(pose_estimation_benchmark ${PROJECT_NAME}
                        benchmark::benchmark benchmark::benchmark_main)

  install(
    TARGETS pose_estimation_benchmark
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

# ##############################################################################
# Install ##
# ##############################################################################

# Export ONLY the library as part of export_${PROJECT_NAME}
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

# Install the node, but do NOT add it to the export set
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

# ##############################################################################
# Export##
# ##############################################################################

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})
ament_export_include_directories(include)

ament_package()
