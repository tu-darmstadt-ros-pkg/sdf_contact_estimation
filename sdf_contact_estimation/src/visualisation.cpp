#include <sdf_contact_estimation/visualisation.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <voxblox_msgs/Mesh.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <eigen_conversions/eigen_msg.h>

namespace sdf_contact_estimation {

void publishMesh(const ros::Publisher &pub, const std::shared_ptr<voxblox::MeshLayer> &mesh, const std::string& frame_id)
{
  voxblox_msgs::Mesh mesh_msg;
  voxblox::generateVoxbloxMeshMsg(mesh, voxblox::ColorMode::kNormals, &mesh_msg);
  mesh_msg.header.frame_id = frame_id;
  pub.publish(mesh_msg);
}

void deleteAllMarkers(const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.header.frame_id = "world"; // this has to be set correctly for the deletion to work
  array.markers.push_back(marker);
  pub.publish(array);
}

void publishTsdfSlice(ros::Publisher &pub, const std::shared_ptr<voxblox::TsdfMap>& tsdf, const Eigen::Isometry3d& pose, const std::string& frame_id, float width)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
  createDistancePointcloudFromTsdfLayer(tsdf->getTsdfLayer(), tsdf_cloud_raw.get());

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(tsdf_cloud_raw);
  pass.setFilterFieldName("y");
  float length = width / 2.0f;
  pass.setFilterLimits(-1*length, length);
  pcl::PointCloud<pcl::PointXYZI> tsdf_cloud_sliced;
  pass.filter(tsdf_cloud_sliced);
  for(auto& p : tsdf_cloud_sliced)
    p.intensity = std::abs(p.intensity);

  Eigen::Affine3d pose_affine(pose);
  pcl::transformPointCloud(tsdf_cloud_sliced, tsdf_cloud_sliced, pose_affine);
  tsdf_cloud_sliced.header.frame_id = frame_id;
  pub.publish(tsdf_cloud_sliced);
}

void publishTsdfSlice(ros::Publisher &pub, const std::shared_ptr<voxblox::TsdfMap> &tsdf, const std::string& frame_id) {
  publishTsdfSlice(pub, tsdf, Eigen::Isometry3d::Identity(), frame_id);
}

void publishEsdfSlice(ros::Publisher& pub, const std::shared_ptr<voxblox::EsdfMap>& esdf, const std::string& frame_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr esdf_cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
  createDistancePointcloudFromEsdfLayer(esdf->getEsdfLayer(), esdf_cloud_raw.get());

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(esdf_cloud_raw);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.1, 0.1);
  pcl::PointCloud<pcl::PointXYZI> esdf_cloud_sliced;
  pass.filter(esdf_cloud_sliced);
  for(auto& p : esdf_cloud_sliced)
    p.intensity = std::abs(p.intensity);
  esdf_cloud_sliced.header.frame_id = frame_id;
  pub.publish(esdf_cloud_sliced);
}

void publishPose(const ros::Publisher &pub, const Eigen::Isometry3d &pose, std::string frame_id) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = frame_id;
  tf::poseEigenToMsg(pose, pose_msg.pose);
  pub.publish(pose_msg);
}

void publishPoint(const ros::Publisher& pub, const Eigen::Vector3d& point, std::string frame_id) {
  geometry_msgs::PointStamped point_msg;
  point_msg.header.frame_id = frame_id;
  tf::pointEigenToMsg(point, point_msg.point);
  pub.publish(point_msg);
}

void publishShape(const RobotShape &robot_shape, const ros::Publisher &pub, const Eigen::Isometry3d &pose, std::string frame_id, const Eigen::Vector3d &color)
{
  visualization_msgs::MarkerArray marker_array;
  for (unsigned int i = 0; i < robot_shape.size(); i++) {
    visualization_msgs::Marker marker = robot_shape[i]->getVisualizationMarker();
    marker.header.frame_id = frame_id;
    marker.ns = "robot_shape";
    marker.id = i;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);

    Eigen::Isometry3d marker_pose = pose * robot_shape[i]->getBaseTransform();
    tf::poseEigenToMsg(marker_pose, marker.pose);
    marker_array.markers.push_back(marker);
    const std::vector<Eigen::Vector3d>& sampling_points = robot_shape[i]->getSamplingPoints();
    for (unsigned int j = 0; j < sampling_points.size(); j++) {
      visualization_msgs::Marker sp_marker;
      sp_marker.type = visualization_msgs::Marker::SPHERE;
      sp_marker.action = visualization_msgs::Marker::ADD;
      sp_marker.scale.x = 0.02;
      sp_marker.scale.y = 0.02;
      sp_marker.scale.z = 0.02;
      sp_marker.color.a = 1.0;
      sp_marker.color.r = 0.0;
      sp_marker.color.g = 1.0;
      sp_marker.color.b = 1.0;

      sp_marker.header.frame_id = frame_id;
      sp_marker.ns = "sampling_points_shape_" + std::to_string(i);
      sp_marker.id = j;

      Eigen::Isometry3d sp_marker_pose = Eigen::Isometry3d::Identity();
      sp_marker_pose.translation() = sampling_points[j];
      sp_marker_pose = pose * sp_marker_pose;
      tf::poseEigenToMsg(sp_marker_pose, sp_marker.pose);
      marker_array.markers.push_back(sp_marker);
    }
  }
  pub.publish(marker_array);
}



}
