#ifndef SDF_CONTACT_ESTIMATION_POSE_OPTIMIZER_H
#define SDF_CONTACT_ESTIMATION_POSE_OPTIMIZER_H

#include <ros/node_handle.h>

#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>

namespace sdf_contact_estimation {

class PoseOptimizer {
public:
  PoseOptimizer(const ros::NodeHandle& nh, const SdfModel& interpolated_sdf, const ShapeModelPtr& shape_model);
  Eigen::Isometry3d doFallingStep(const Eigen::Isometry3d& com_pose, const Eigen::Isometry3d &base_to_com) const;
  Eigen::Isometry3d doRotationStep(const Eigen::Isometry3d& com_pose, const Eigen::Isometry3d &base_to_com, const Eigen::Isometry3d& tipping_frame) const;
private:
  const SdfModel* interpolated_sdf_;
  ShapeModelPtr shape_model_;
};

}
#endif
