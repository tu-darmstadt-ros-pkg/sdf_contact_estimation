#ifndef SDF_CONTACT_ESTIMATION_SHAPE_COLLISION_TYPES_H
#define SDF_CONTACT_ESTIMATION_SHAPE_COLLISION_TYPES_H

#include <std_msgs/msg/color_rgba.hpp>

namespace sdf_contact_estimation
{

// MAIN_TRACK is a subtype of TRACK: it behaves like a track for contact /
// stability (is_track_ stays true) but is additionally flagged is_main_track_ so
// callers can tell the robot's main tracks apart from its flippers.
enum CollisionType { DEFAULT, TRACK, BODY, MAIN_TRACK };

struct SamplingInfo {
  double resolution;
  double cylinder_angle_min;
  double cylinder_angle_max;
  bool invert_x = false;
  bool invert_y = false;
  bool invert_z = false;
};

struct CollisionInfo {
  std::string link_name;
  CollisionType type;
  SamplingInfo sampling_info;
  std::vector<int> ignore_indices;
  std::vector<int> include_indices;
  std::vector<int> invert_x_indices;
  std::vector<int> invert_y_indices;
  std::vector<int> invert_z_indices;
};

CollisionType stringToCollisionType( const std::string &str );

std_msgs::msg::ColorRGBA collisionTypeToColor( const CollisionType &type );
} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_SHAPE_COLLISION_TYPES_H
