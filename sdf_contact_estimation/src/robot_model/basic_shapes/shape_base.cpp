#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

namespace sdf_contact_estimation {

ShapeBase::ShapeBase(const Eigen::Isometry3d& base_transform, const SamplingInfo& sampling_info, bool is_track, bool is_body)
  : base_transform_(base_transform), sampling_info_(sampling_info), is_track_(is_track), is_body_(is_body)
{}

ShapeBase::~ShapeBase() = default;

const Eigen::Isometry3d& ShapeBase::getBaseTransform() const {
  return base_transform_;
}

void ShapeBase::setBaseTransform(const Eigen::Isometry3d& transform) {
  base_transform_ = transform;
}

double ShapeBase::getSamplingResolution() const
{
  return sampling_info_.resolution;
}

size_t ShapeBase::getSamplingPointsCount() const
{
  return sampling_points_.size();
}

std::vector<Eigen::Vector3d> ShapeBase::getSamplingPoints() const{
  return transformSamplingPointsToBase();
}

bool ShapeBase::isTrack() const {
  return is_track_;
}

void ShapeBase::setTrack(bool is_track) {
  is_track_ = is_track;
}

bool ShapeBase::isBody() const {
  return is_body_;
}

void ShapeBase::setBody(bool is_body) {
  is_body_ = is_body;
}

std::vector<Eigen::Vector3d> ShapeBase::transformSamplingPointsToBase() const{
  std::vector<Eigen::Vector3d> sampling_points_base;
  sampling_points_base.reserve(sampling_points_.size());
  for (const Eigen::Vector3d& sampling_point: sampling_points_) {
    sampling_points_base.emplace_back(base_transform_ * sampling_point);
  }
  return sampling_points_base;
}


}
