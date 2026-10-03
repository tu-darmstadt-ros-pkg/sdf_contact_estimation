#ifndef SDF_CONTACT_ESTIMATION_RECTANGLE_SHAPE_H
#define SDF_CONTACT_ESTIMATION_RECTANGLE_SHAPE_H

#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>

namespace sdf_contact_estimation
{

class RectangleShape : public ShapeBase
{
public:
  RectangleShape( double length_x, double length_y, const Eigen::Isometry3d &base_transform,
                  const SamplingInfo &sampling_info, bool is_track = false, bool is_body = false )
      : ShapeBase( base_transform, sampling_info, is_track, is_body ), length_x_( length_x ),
        length_y_( length_y )
  {
    sampling_points_ = generateSamplingPoints();
  }

  /**
   * @brief getSamplingPoints Returns a list of points, that sample this shape. The points are given relative to the shape.
   * @return List of points that sample this shape
   */
  std::vector<Eigen::Vector3d> generateSamplingPoints()
  {
    std::vector<Eigen::Vector3d> sampling_points;

    // The fixed-step grid is flush against the edge it starts from and leaves a
    // gap at the far edge when the size is not an integer multiple of the
    // resolution. invert_x / invert_y start counting from the +edge instead of
    // the -edge so the flush (sampled) edge can be chosen to be the OUTER one,
    // letting the support polygon capture the outermost contact points.
    const double res = sampling_info_.resolution;
    for ( double x = -length_x_ / 2.0; x <= length_x_ / 2.0; x += res ) {
      for ( double y = -length_y_ / 2.0; y <= length_y_ / 2.0; y += res ) {
        const double px = sampling_info_.invert_x ? -x : x;
        const double py = sampling_info_.invert_y ? -y : y;
        sampling_points.emplace_back( px, py, 0 );
      }
    }

    return sampling_points;
  }

  visualization_msgs::msg::Marker getVisualizationMarker() override
  {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = length_x_;
    marker.scale.y = length_y_;
    marker.scale.z = 0.001;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
  }

private:
  double length_x_;
  double length_y_;
};

} // namespace sdf_contact_estimation

#endif
