#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>

namespace sdf_contact_estimation
{

ShapeBase::ShapeBase( const Eigen::Isometry3d &base_transform, const SamplingInfo &sampling_info,
                      bool is_track, bool is_body, bool is_main_track )
    : base_transform_( base_transform ), sampling_info_( sampling_info ), is_track_( is_track ),
      is_body_( is_body ), is_main_track_( is_main_track )
{
}

ShapeBase::~ShapeBase() = default;

const Eigen::Isometry3d &ShapeBase::getBaseTransform() const { return base_transform_; }

void ShapeBase::setBaseTransform( const Eigen::Isometry3d &transform )
{
  base_transform_ = transform;
  base_transform_changed_ = true; // invalidate the cached base-frame points
}

double ShapeBase::getSamplingResolution() const { return sampling_info_.resolution; }

size_t ShapeBase::getSamplingPointsCount() const { return sampling_points_.size(); }

const std::vector<Eigen::Vector3d> &ShapeBase::getSamplingPoints() const
{
  if ( base_transform_changed_ )
    transformSamplingPointsToBase();
  return sampling_points_base_;
}

bool ShapeBase::isTrack() const { return is_track_; }

void ShapeBase::setTrack( bool is_track ) { is_track_ = is_track; }

bool ShapeBase::isBody() const { return is_body_; }

void ShapeBase::setBody( bool is_body ) { is_body_ = is_body; }

bool ShapeBase::isMainTrack() const { return is_main_track_; }

void ShapeBase::setMainTrack( bool is_main_track ) { is_main_track_ = is_main_track; }

void ShapeBase::transformSamplingPointsToBase() const
{
  sampling_points_base_.resize( sampling_points_.size() );
  for ( size_t i = 0; i < sampling_points_.size(); ++i )
    sampling_points_base_[i] = base_transform_ * sampling_points_[i];
  base_transform_changed_ = false;
}

} // namespace sdf_contact_estimation
