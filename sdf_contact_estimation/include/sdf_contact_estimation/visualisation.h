#ifndef SDF_CONTACT_ESTIMATION_VISUALISATION_H
#define SDF_CONTACT_ESTIMATION_VISUALISATION_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <Eigen/Eigen>
#include <pcl/common/common.h>
#include <pcl/Vertices.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox_msgs/msg/mesh.hpp>

#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>

namespace sdf_contact_estimation
{

// Robot shape and markers
void publishShape(
  const RobotShape & robot_shape,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const Eigen::Isometry3d & pose,
  const std::string & frame_id,
  const Eigen::Vector3d & color);

void deleteAllMarkers(
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub);

// Mesh
void publishMesh(
  const rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr & pub,
  const std::shared_ptr<voxblox::MeshLayer> & mesh,
  const std::string & frame_id);

// TSDF / ESDF slices (published as PointCloud2)
void publishTsdfSlice(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  const std::shared_ptr<voxblox::TsdfMap> & tsdf,
  const std::string & frame_id);

void publishTsdfSlice(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  const std::shared_ptr<voxblox::TsdfMap> & tsdf,
  const Eigen::Isometry3d & pose,
  const std::string & frame_id,
  float width = 0.2f);

void publishEsdfSlice(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  const std::shared_ptr<voxblox::EsdfMap> & esdf,
  const std::string & frame_id);

// Pose / point
void publishPose(
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
  const Eigen::Isometry3d & pose,
  const std::string & frame_id);

void publishPoint(
  const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & pub,
  const Eigen::Vector3d & point,
  const std::string & frame_id);

}  // namespace sdf_contact_estimation

#endif  // SDF_CONTACT_ESTIMATION_VISUALISATION_H
