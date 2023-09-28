//
// Created by martin on 23.02.22.
//
#include <sdf_contact_estimation/sdf/sdf_model.h>

#include <Eigen/Eigen>
#include <vector>

#ifndef SDF_CONTACT_ESTIMATION_FALLING_POSE_OPTIMIZER_H
#define SDF_CONTACT_ESTIMATION_FALLING_POSE_OPTIMIZER_H

namespace sdf_contact_estimation {

class FallingPoseOptimizer {
public:
  explicit FallingPoseOptimizer(const SdfModel& interpolated_sdf, const std::vector<Eigen::Vector3d>& sampling_points);
  void optimize(Eigen::Isometry3d& world_to_com) const;

private:
  void evaluate(const Eigen::Isometry3d& world_to_com, double& gradient, bool& valid_solution, bool& active_constraint) const;
  const SdfModel& interpolated_sdf_;
  const std::vector<Eigen::Vector3d>& sampling_points_;
};
}



#endif //SDF_CONTACT_ESTIMATION_FALLING_POSE_OPTIMIZER_H
