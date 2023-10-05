//
// Created by martin on 23.02.22.
//

#include "sdf_contact_estimation/optimization/falling_pose_optimizer.h"

#include <chrono>
#include <ros/console.h>
namespace sdf_contact_estimation {

FallingPoseOptimizer::FallingPoseOptimizer(const SdfModel& interpolated_sdf, const std::vector<Eigen::Vector3d>& sampling_points)
 : interpolated_sdf_(interpolated_sdf), sampling_points_(sampling_points) {}

void FallingPoseOptimizer::optimize(Eigen::Isometry3d& world_to_com) const {
  auto start = std::chrono::high_resolution_clock::now();

  size_t max_num_iterations = 10;
  double step_size = 1.0;
  double step_size_factor = 0.8;
  unsigned int iteration_counter;
  for (iteration_counter = 1; iteration_counter <= max_num_iterations; ++iteration_counter) {
    bool valid_solution, active_constraint;
    double gradient;
    evaluate(world_to_com, gradient, valid_solution, active_constraint);
    ROS_DEBUG_STREAM(iteration_counter << ": height: " << world_to_com.translation().z() << ", gradient: " <<
    gradient << ", valid: " << valid_solution << ", active constraints: " << active_constraint);

    // repeat until all constraints are fulfilled and at least one constraint is active (close to 0)
    if (valid_solution && active_constraint) {
      ROS_DEBUG_STREAM("Solution found, aborting optimization");
      break;
    }

    // Update parameter
    // move down (or up) the smallest distance
    world_to_com.translation().z() += step_size * gradient;
    step_size *= step_size_factor; // decrease step size to prevent oscillation
  }

  if (iteration_counter > max_num_iterations) {
    ROS_DEBUG_STREAM("Max iterations reached");
  }

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  ROS_DEBUG_STREAM("Falling optimization took " << iteration_counter << " evaluations in " << elapsed.count()*1000.0 << " ms.");
}

void FallingPoseOptimizer::evaluate(const Eigen::Isometry3d &world_to_com, double &gradient, bool& valid_solution, bool& active_constraint) const {
  // Collect distance to ground of all sampling spheres
  double min_distance = std::numeric_limits<double>::max();
  valid_solution = true;
  active_constraint = false;
  // Iterate through all points and update distances
  for (const Eigen::Vector3d& sampling_point: sampling_points_) {
    Eigen::Vector3d sampling_point_world = world_to_com * sampling_point;
    double distance = interpolated_sdf_.getSdf(sampling_point_world(0), sampling_point_world(1),
                                               sampling_point_world(2));

    // Update smallest distance
    if (distance < min_distance) {
      min_distance = distance;
    }

    // Check if valid
    if (distance < -1e-3) {
      valid_solution = false;
    }
    // Check if active
    if (std::abs(distance) < 0.02) {
      active_constraint = true;
    }

  }

  if (min_distance == std::numeric_limits<double>::max()) {
    ROS_DEBUG_STREAM("Could not find minimum distance to ground.");
    min_distance = 0;
  }

  gradient = -min_distance;

}

}