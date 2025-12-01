#include <sdf_contact_estimation/visualisation.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <voxblox_msgs/msg/mesh.hpp>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sdf_contact_estimation
{

void publishMesh(
  const rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr & pub,
  const std::shared_ptr<voxblox::MeshLayer> & mesh,
  const std::string & frame_id)
{
  if (!pub) {
    return;
  }

  voxblox_msgs::msg::Mesh mesh_msg;
  voxblox::generateVoxbloxMeshMsg(mesh, voxblox::ColorMode::kNormals, &mesh_msg);
  mesh_msg.header.frame_id = frame_id;
  pub->publish(mesh_msg);
}

void deleteAllMarkers(
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub)
{
  if (!pub) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  // Needs a valid frame for RViz to accept the deletion
  marker.header.frame_id = "world";
  array.markers.push_back(marker);
  pub->publish(array);
}

void publishTsdfSlice(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  const std::shared_ptr<voxblox::TsdfMap> & tsdf,
  const Eigen::Isometry3d & pose,
  const std::string & frame_id,
  float width)
{
  if (!pub || !tsdf) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_raw(
    new pcl::PointCloud<pcl::PointXYZI>);
  createDistancePointcloudFromTsdfLayer(
    tsdf->getTsdfLayer(), tsdf_cloud_raw.get());

  // Slice in y-direction
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(tsdf_cloud_raw);
  pass.setFilterFieldName("y");
  float length = width / 2.0f;
  pass.setFilterLimits(-1.0f * length, length);

  pcl::PointCloud<pcl::PointXYZI> tsdf_cloud_sliced;
  pass.filter(tsdf_cloud_sliced);
  for (auto & p : tsdf_cloud_sliced) {
    p.intensity = std::abs(p.intensity);
  }

  Eigen::Affine3d pose_affine(pose);
  pcl::transformPointCloud(tsdf_cloud_sliced, tsdf_cloud_sliced, pose_affine);

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(tsdf_cloud_sliced, msg);
  msg.header.frame_id = frame_id;
  // (stamp can be set by the caller’s node if needed)
  pub->publish(msg);
}

void publishTsdfSlice(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  const std::shared_ptr<voxblox::TsdfMap> & tsdf,
  const std::string & frame_id)
{
  publishTsdfSlice(pub, tsdf, Eigen::Isometry3d::Identity(), frame_id);
}

void publishEsdfSlice(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  const std::shared_ptr<voxblox::EsdfMap> & esdf,
  const std::string & frame_id)
{
  if (!pub || !esdf) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr esdf_cloud_raw(
    new pcl::PointCloud<pcl::PointXYZI>);
  createDistancePointcloudFromEsdfLayer(
    esdf->getEsdfLayer(), esdf_cloud_raw.get());

  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(esdf_cloud_raw);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.1, 0.1);

  pcl::PointCloud<pcl::PointXYZI> esdf_cloud_sliced;
  pass.filter(esdf_cloud_sliced);
  for (auto & p : esdf_cloud_sliced) {
    p.intensity = std::abs(p.intensity);
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(esdf_cloud_sliced, msg);
  msg.header.frame_id = frame_id;
  pub->publish(msg);
}

void publishPose(
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
  const Eigen::Isometry3d & pose,
  const std::string & frame_id)
{
  if (!pub) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id;
  // pose_msg.header.stamp can be set by caller if needed
  pose_msg.pose = tf2::toMsg(pose);
  pub->publish(pose_msg);
}

void publishPoint(
  const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & pub,
  const Eigen::Vector3d & point,
  const std::string & frame_id)
{
  if (!pub) {
    return;
  }

  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.frame_id = frame_id;
  point_msg.point = tf2::toMsg(point);
  pub->publish(point_msg);
}

void publishShape(
  const RobotShape & robot_shape,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const Eigen::Isometry3d & pose,
  const std::string & frame_id,
  const Eigen::Vector3d & color)
{
  if (!pub) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  for (unsigned int i = 0; i < robot_shape.size(); ++i) {
    visualization_msgs::msg::Marker marker =
      robot_shape[i]->getVisualizationMarker();
    marker.header.frame_id = frame_id;
    marker.ns = "robot_shape";
    marker.id = static_cast<int32_t>(i);
    marker.color.r = static_cast<float>(color(0));
    marker.color.g = static_cast<float>(color(1));
    marker.color.b = static_cast<float>(color(2));

    Eigen::Isometry3d marker_pose = pose * robot_shape[i]->getBaseTransform();
    marker.pose = tf2::toMsg(marker_pose);
    marker_array.markers.push_back(marker);

    const std::vector<Eigen::Vector3d> & sampling_points =
      robot_shape[i]->getSamplingPoints();
    for (unsigned int j = 0; j < sampling_points.size(); ++j) {
      visualization_msgs::msg::Marker sp_marker;
      sp_marker.type = visualization_msgs::msg::Marker::SPHERE;
      sp_marker.action = visualization_msgs::msg::Marker::ADD;
      sp_marker.scale.x = 0.02;
      sp_marker.scale.y = 0.02;
      sp_marker.scale.z = 0.02;
      sp_marker.color.a = 1.0;
      sp_marker.color.r = 0.0;
      sp_marker.color.g = 1.0;
      sp_marker.color.b = 1.0;

      sp_marker.header.frame_id = frame_id;
      sp_marker.ns = "sampling_points_shape_" + std::to_string(i);
      sp_marker.id = static_cast<int32_t>(j);

      Eigen::Isometry3d sp_marker_pose = Eigen::Isometry3d::Identity();
      sp_marker_pose.translation() = sampling_points[j];
      sp_marker_pose = pose * sp_marker_pose;
      sp_marker.pose = tf2::toMsg(sp_marker_pose);

      marker_array.markers.push_back(sp_marker);
    }
  }

  pub->publish(marker_array);
}

}  // namespace sdf_contact_estimation
