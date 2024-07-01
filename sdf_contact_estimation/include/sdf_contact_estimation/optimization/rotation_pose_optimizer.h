//
// Created by martin on 23.02.22.
//

#ifndef SDF_CONTACT_ESTIMATION_ROTATION_POSE_OPTIMIZER_H
#define SDF_CONTACT_ESTIMATION_ROTATION_POSE_OPTIMIZER_H

#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <Eigen/Eigen>

namespace sdf_contact_estimation {
class RotationPoseOptimizer {
public:
  RotationPoseOptimizer(const SdfModel& interpolated_sdf,
                        const std::vector<Eigen::Vector3d>& sampling_points,
                        const Eigen::Isometry3d& rotation_frame,
                        bool positive_rotation_direction);
  void optimize(double& rotation_angle) const;

  double getContactThreshold() const;
  void setContactThreshold(double contact_threshold);
private:
  void evaluate(double rotation_angle, double& gradient, bool& valid_solution, bool& active_constraint) const;

  const SdfModel& interpolated_sdf_;
  const std::vector<Eigen::Vector3d>& sampling_points_;
  const Eigen::Isometry3d rotation_frame_;
  bool positive_rotation_direction_;
  double contact_threshold_{0.02};
};
}



#endif //SDF_CONTACT_ESTIMATION_ROTATION_POSE_OPTIMIZER_H
