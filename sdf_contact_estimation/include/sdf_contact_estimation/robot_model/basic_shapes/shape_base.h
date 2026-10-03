#ifndef SDF_CONTACT_ESTIMATION_SHAPE_BASE_H
#define SDF_CONTACT_ESTIMATION_SHAPE_BASE_H

#include <Eigen/Eigen>
#include <sdf_contact_estimation/robot_model/shape_collision_types.h>
#include <visualization_msgs/msg/marker.hpp>

namespace sdf_contact_estimation
{

class ShapeBase
{
public:
  ShapeBase( const Eigen::Isometry3d &base_transform, const SamplingInfo &sampling_info,
             bool is_track = false, bool is_body = false, bool is_main_track = false );
  virtual ~ShapeBase();

  virtual visualization_msgs::msg::Marker getVisualizationMarker() = 0;

  const Eigen::Isometry3d &getBaseTransform() const;
  void setBaseTransform( const Eigen::Isometry3d &transform );

  double getSamplingResolution() const;
  size_t getSamplingPointsCount() const;

  /// Sampling points transformed into the base frame. Cached and only recomputed
  /// when the base transform changes (setBaseTransform), so repeated calls within
  /// one optimizer step / contact pass do not reallocate or re-transform. Returned
  /// by const reference; the reference is valid until the next setBaseTransform().
  const std::vector<Eigen::Vector3d> &getSamplingPoints() const;

  bool isTrack() const;
  void setTrack( bool is_track );

  bool isBody() const;
  void setBody( bool is_body );

  /// Whether this shape belongs to one of the robot's MAIN tracks (as opposed to
  /// a flipper). A main-track shape is always also a track (isTrack() == true).
  bool isMainTrack() const;
  void setMainTrack( bool is_main_track );

protected:
  void transformSamplingPointsToBase() const; // (re)fills sampling_points_base_

  Eigen::Isometry3d
      base_transform_; // Transformation from base_frame to this shape (transforms points from this shape to base)
  SamplingInfo sampling_info_;
  std::vector<Eigen::Vector3d> sampling_points_; // in the shape's own frame
  // Cache of sampling_points_ transformed by base_transform_. Lazily rebuilt
  // when base_transform_changed_ is set (see setBaseTransform / getSamplingPoints).
  mutable std::vector<Eigen::Vector3d> sampling_points_base_;
  mutable bool base_transform_changed_ = true;
  bool is_track_;
  bool is_body_;
  bool is_main_track_;
};

typedef std::shared_ptr<ShapeBase> ShapePtr;
typedef std::vector<ShapePtr> RobotShape;

} // namespace sdf_contact_estimation

#endif
