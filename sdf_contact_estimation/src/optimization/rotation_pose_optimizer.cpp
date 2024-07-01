//
// Created by martin on 23.02.22.
//

#include "sdf_contact_estimation/optimization/rotation_pose_optimizer.h"

namespace sdf_contact_estimation {

RotationPoseOptimizer::RotationPoseOptimizer(const SdfModel& interpolated_sdf,
                                             const std::vector<Eigen::Vector3d>& sampling_points,
                                             const Eigen::Isometry3d& rotation_frame,
                                             bool positive_rotation_direction)
: interpolated_sdf_(interpolated_sdf),
  sampling_points_(sampling_points),
  rotation_frame_(rotation_frame),
  positive_rotation_direction_(positive_rotation_direction)
  {}

void RotationPoseOptimizer::optimize(double& rotation_angle) const {
  size_t max_num_iterations = 10;
  double step_size = 1.0;
  double step_size_factor = 0.8;
  unsigned int iteration_counter;
  auto start = std::chrono::high_resolution_clock::now();
  for (iteration_counter = 1; iteration_counter <= max_num_iterations; ++iteration_counter) {
    bool valid_solution, active_constraint;
    double gradient;
    evaluate(rotation_angle, gradient, valid_solution, active_constraint);
    ROS_DEBUG_STREAM(iteration_counter << ": angle: " << rotation_angle << ", gradient: " <<
                                       gradient << ", valid: " << valid_solution << ", active constraints: " << active_constraint);

    // repeat until all constraints are fulfilled and at least one constraint is active (close to 0)
    if (valid_solution && active_constraint) {
      ROS_DEBUG_STREAM("Solution found, aborting");
      break;
    }

    // Update parameter
    // move down (or up) the smallest distance
    if (positive_rotation_direction_) {
      rotation_angle += step_size * gradient;
    } else {
      rotation_angle -= step_size * gradient;
    }
    step_size *= step_size_factor;
  }

  if (iteration_counter > max_num_iterations) {
    ROS_DEBUG_STREAM("Max iterations reached");
  }
  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_DEBUG_STREAM("Tipping optimization took " << iteration_counter << " evaluations in " << elapsed.count()*1000.0 << " ms.");
}

void RotationPoseOptimizer::evaluate(double rotation_angle, double& gradient, bool& valid_solution, bool& active_constraint) const {
  // Collect distance to ground of all sampling spheres
  double min_value = std::numeric_limits<double>::max();
  valid_solution = true;
  active_constraint = false;
  // Iterate through all points and update distances
  Eigen::Isometry3d rotation_matrix(Eigen::AngleAxisd(rotation_angle, Eigen::Vector3d::UnitX()));
  Eigen::Isometry3d rotated_frame = rotation_frame_ * rotation_matrix;
  for (const Eigen::Vector3d& sampling_point: sampling_points_) {
    Eigen::Vector3d sampling_point_world = rotated_frame * sampling_point;
    double distance = interpolated_sdf_.getSdf(sampling_point_world(0), sampling_point_world(1),
                                               sampling_point_world(2));

    Eigen::Vector3d sampling_point_old_tipping_frame = rotation_matrix * sampling_point;
//    Eigen::Vector3d sampling_point_proj(std::abs(sampling_point_old_tipping_frame.y()), sampling_point_old_tipping_frame.z(), 0);
//    Eigen::Vector3d contact_point_proj(std::abs(sampling_point_old_tipping_frame.y()), sampling_point_old_tipping_frame.z() - distance, 0);
//    double d = sampling_point_proj.norm();
//    double c = contact_point_proj.norm();

    double y_abs = std::abs(sampling_point_old_tipping_frame.y());
    double y_abs_2 = y_abs*y_abs;
    double d_2 = y_abs_2 + sampling_point_old_tipping_frame.z() * sampling_point_old_tipping_frame.z();
    double distance_2 = distance*distance;
    double C = sampling_point_old_tipping_frame.z() - distance;
    double c_2 = y_abs_2 + C*C;
    double bc2 = 2 * std::sqrt(d_2 * c_2);

    double sign = distance > 0 ? 1.0 : -1.0;

    // We want to calculate the minimum angle, but atan is expensive.
    // Since the function is monotone, we can just use the argument for comparison
//    double s = (d + c + distance) / 2;
//    double value = sign * (s - d) * (s - c) / (s * (s - distance));
    double value = sign * (2*bc2/(-distance_2 + d_2 + bc2 + c_2) - 1);
    
    // For a valid solution, all points have to be above the ground
    if (distance < -1e-3) {
      valid_solution = false;
    }
    // Check if constraint is active -> point is close to the surface
    if (std::abs(distance) < contact_threshold_) {
      active_constraint = true;
    }
    // Update smallest angle
    if (value < min_value) {
      min_value = value;
    }
  }

  if (min_value == std::numeric_limits<double>::max()) {
    ROS_DEBUG_STREAM("Could not find minimum rotation angle.");
    min_value = 0;
  }

  double sign = min_value > 0 ? 1.0 : - 1.0;
  gradient = sign * 2*std::atan((std::sqrt(sign * min_value)));
}
double RotationPoseOptimizer::getContactThreshold() const {
  return contact_threshold_;
}
void RotationPoseOptimizer::setContactThreshold(double contact_threshold) {
  contact_threshold_ = contact_threshold;
}

}