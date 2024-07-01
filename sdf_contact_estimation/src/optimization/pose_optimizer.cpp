#include <sdf_contact_estimation/optimization/pose_optimizer.h>

#include <sdf_contact_estimation/util/timing.h>

#include <sdf_contact_estimation/optimization/falling_pose_optimizer.h>
#include <sdf_contact_estimation/optimization/rotation_pose_optimizer.h>

#include <functional>
#include <chrono>

namespace sdf_contact_estimation {

PoseOptimizer::PoseOptimizer(const ros::NodeHandle& nh, const SdfModel &interpolated_sdf, const ShapeModelPtr& shape_model, double contact_threshold)
  : interpolated_sdf_(&interpolated_sdf), shape_model_(shape_model), contact_threshold_(contact_threshold)
{}

  START_TIMING("PoseOptimizer::doFallingStep")
Eigen::Isometry3d PoseOptimizer::doFallingStep(const Eigen::Isometry3d &com_pose, const Eigen::Isometry3d& base_to_com) const {
  // Set up problem
  Eigen::Isometry3d com_to_base = base_to_com.inverse();

  // Collect sampling points and transform to COM frame
  std::vector<Eigen::Vector3d> sampling_points;
  sampling_points.reserve(shape_model_->getTotalSamplingPointCount());
  for (const ShapePtr& shape: shape_model_->getShape()) {
    std::vector<Eigen::Vector3d> sampling_points_base = shape->getSamplingPoints();
    for (const Eigen::Vector3d& sampling_point_base: sampling_points_base) {
      Eigen::Vector3d sampling_point_com = com_to_base * sampling_point_base;
      sampling_points.emplace_back(sampling_point_com);
    }
  }

  if (sampling_points.empty()) {
    ROS_DEBUG_STREAM("No sampling points in falling step");
    STOP_TIMING_AVG
    return {};
  }

  FallingPoseOptimizer optimizer(*interpolated_sdf_, sampling_points);
  optimizer.setContactThreshold(contact_threshold_);

  // Initialize solution
  Eigen::Isometry3d result_world_to_com = com_pose;

  // Iterate
  optimizer.optimize(result_world_to_com);

  STOP_TIMING_AVG
  return result_world_to_com;
}

Eigen::Isometry3d PoseOptimizer::doRotationStep(const Eigen::Isometry3d &com_pose, const Eigen::Isometry3d &base_to_com, const Eigen::Isometry3d &tipping_frame) const
{
  START_TIMING("PoseOptimizer::doRotationStep")
  // Set up problem
  Eigen::Isometry3d tipping_to_com = tipping_frame.inverse() * com_pose;
  Eigen::Isometry3d tipping_to_base = tipping_to_com * base_to_com.inverse();

  bool positive_rotation_direction = (tipping_to_com.translation().y() < 0);
  ROS_DEBUG_STREAM("Rotating in positive direction: " << positive_rotation_direction);

  std::vector<Eigen::Vector3d> sampling_points;
  sampling_points.reserve(shape_model_->getTotalSamplingPointCount());
  for (const ShapePtr& shape: shape_model_->getShape()) {
    std::vector<Eigen::Vector3d> sampling_points_base = shape->getSamplingPoints();
    for (const Eigen::Vector3d& sampling_point_base: sampling_points_base) {
      Eigen::Vector3d sampling_point_tipping = tipping_to_base * sampling_point_base;
      // Ignore all points, that are on the wrong side of the rotation axis
      if ((positive_rotation_direction && sampling_point_tipping.y() < -0.05)
      || (!positive_rotation_direction && sampling_point_tipping.y() > 0.05))
      sampling_points.emplace_back(sampling_point_tipping);
    }
  }

  if (sampling_points.empty()) {
    ROS_DEBUG_STREAM("No sampling points in rotation step");
    STOP_TIMING_AVG
    return {};
  }


  RotationPoseOptimizer optimizer(*interpolated_sdf_, sampling_points, tipping_frame, positive_rotation_direction);
  optimizer.setContactThreshold(contact_threshold_);

  double rotation_angle = 0;
  optimizer.optimize(rotation_angle);

  Eigen::Isometry3d result_world_to_com = tipping_frame * Eigen::AngleAxisd(rotation_angle, Eigen::Vector3d::UnitX())  * tipping_to_com;

  STOP_TIMING_AVG
  return result_world_to_com;
}



}
