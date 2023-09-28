#ifndef SDF_CONTACT_ESTIMATION_SHAPE_BASE_H
#define SDF_CONTACT_ESTIMATION_SHAPE_BASE_H

#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <sdf_contact_estimation/robot_model/shape_collision_types.h>


namespace sdf_contact_estimation {

class ShapeBase {
public:
  ShapeBase(const Eigen::Isometry3d& base_transform, const SamplingInfo& sampling_info, bool is_track=false, bool is_body=false);
  virtual ~ShapeBase();

  virtual visualization_msgs::Marker getVisualizationMarker() = 0;

  const Eigen::Isometry3d& getBaseTransform() const;
  void setBaseTransform(const Eigen::Isometry3d& transform);

  double getSamplingResolution() const;
  size_t getSamplingPointsCount() const;

  std::vector<Eigen::Vector3d> getSamplingPoints() const;

  bool isTrack() const;
  void setTrack(bool is_track);

  bool isBody() const;
  void setBody(bool is_body);
protected:
  std::vector<Eigen::Vector3d> transformSamplingPointsToBase() const;

  Eigen::Isometry3d base_transform_; // Transformation from base_frame to this shape (transforms points from this shape to base)
  SamplingInfo sampling_info_;
  std::vector<Eigen::Vector3d> sampling_points_;
  bool is_track_;
  bool is_body_;
};

typedef std::shared_ptr<ShapeBase> ShapePtr;
typedef std::vector<ShapePtr> RobotShape;

}

#endif
