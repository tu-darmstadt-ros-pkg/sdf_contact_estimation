#ifndef SDF_CONTACT_ESTIMATION_RECTANGLE_SHAPE_H
#define SDF_CONTACT_ESTIMATION_RECTANGLE_SHAPE_H

#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>

namespace sdf_contact_estimation {

class RectangleShape : public ShapeBase {
public:
  RectangleShape(double length_x, double length_y, const Eigen::Isometry3d& base_transfom, const SamplingInfo& sampling_info,
                 bool is_track=false, bool is_body=false)
    : ShapeBase(base_transfom, sampling_info, is_track, is_body), length_x_(length_x), length_y_(length_y)
  {
    sampling_points_ = generateSamplingPoints();
  }

  /**
   * @brief getSamplingPoints Returns a list of points, that sample this shape. The points are given relative to the shape.
   * @return List of points that sample this shape
   */
  std::vector<Eigen::Vector3d> generateSamplingPoints() {
    std::vector<Eigen::Vector3d> sampling_points;

    for (double x = -length_x_/2.0; x <= length_x_/2.0; x += sampling_info_.resolution) {
      for (double y = -length_y_/2.0; y <= length_y_/2.0; y += sampling_info_.resolution) {
        Eigen::Vector3d point(x, y, 0);
        sampling_points.push_back(point);
      }
//      // Make sure to sample last line
//      Eigen::Vector3d point(x, length_y_, 0);
//      Eigen::Vector3d point_base = base_transfom_ * point;
//      sampling_points.push_back(point_base);
    }

    return sampling_points;
  }

  visualization_msgs::Marker getVisualizationMarker() override {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
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

}

#endif
