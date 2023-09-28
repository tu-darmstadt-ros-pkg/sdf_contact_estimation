#ifndef SDF_CONTACT_ESTIMATION_VISUALISATION_H
#define SDF_CONTACT_ESTIMATION_VISUALISATION_H

#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <pcl/common/common.h>
#include <pcl/Vertices.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/mesh/mesh_layer.h>

#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>

namespace sdf_contact_estimation {

void publishShape(const RobotShape& robot_shape, const ros::Publisher& pub, const Eigen::Isometry3d& pose, std::string frame_id, const Eigen::Vector3d& color);
void deleteAllMarkers(const ros::Publisher& pub);
void publishMesh(const ros::Publisher& pub, const std::shared_ptr<voxblox::MeshLayer>& mesh, const std::string& frame_id);
void publishTsdfSlice(ros::Publisher& pub, const std::shared_ptr<voxblox::TsdfMap>& tsdf, const std::string& frame_id);
void publishTsdfSlice(ros::Publisher& pub, const std::shared_ptr<voxblox::TsdfMap>& tsdf, const Eigen::Isometry3d& pose, const std::string& frame_id, float width=0.2);
void publishEsdfSlice(ros::Publisher& pub, const std::shared_ptr<voxblox::EsdfMap>& esdf, const std::string& frame_id);
void publishPose(const ros::Publisher& pub, const Eigen::Isometry3d& pose, std::string frame_id);
void publishPoint(const ros::Publisher& pub, const Eigen::Vector3d& point, std::string frame_id);

}

#endif
