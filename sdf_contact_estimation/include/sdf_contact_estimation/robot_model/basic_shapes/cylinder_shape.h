#ifndef SDF_CONTACT_ESTIMATION__CYLINDER_SHAPE_H
#define SDF_CONTACT_ESTIMATION__CYLINDER_SHAPE_H

#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>

namespace sdf_contact_estimation {

class CylinderShape : public ShapeBase {
public:
  CylinderShape(double radius, double height, const Eigen::Isometry3d& base_transform, const SamplingInfo& sampling_info,
                bool is_track=false, bool is_body=false)
    : ShapeBase(base_transform, sampling_info, is_track, is_body), radius_(radius), height_(height)
  {
    sampling_points_ = generateSamplingPoints();
  }

  /**
   * @brief getSamplingPoints Returns a list of points, that sample this shape. The points are given relative to the shape.
   * @return List of points that sample this shape
   */
  std::vector<Eigen::Vector3d> generateSamplingPoints() {
    std::vector<Eigen::Vector3d> sampling_points;

    double circumference = (sampling_info_.cylinder_angle_max - sampling_info_.cylinder_angle_min) * radius_;
    int circle_sampling_count = std::floor(circumference / sampling_info_.resolution);
    double angle_step = (sampling_info_.cylinder_angle_max - sampling_info_.cylinder_angle_min) / static_cast<double>(circle_sampling_count);

    int height_sampling_count = std::floor(height_ / sampling_info_.resolution);

    for (unsigned int i = 0; i < circle_sampling_count; i++) {
      for (unsigned int j = 0; j <= height_sampling_count; j++) {
        Eigen::Vector3d point;
        double angle = sampling_info_.cylinder_angle_min + i * angle_step;
        point.x() = radius_ * std::cos(angle);
        point.y() = radius_ * std::sin(angle);

        point.z() = j * sampling_info_.resolution - height_/2.0;

        sampling_points.push_back(point);
      }
    }

    return sampling_points;
  }

  visualization_msgs::Marker getVisualizationMarker() override{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 2*radius_;
    marker.scale.y = 2*radius_;
    marker.scale.z = height_;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
  }

private:
  double radius_;
  double height_;
};

}

#endif
