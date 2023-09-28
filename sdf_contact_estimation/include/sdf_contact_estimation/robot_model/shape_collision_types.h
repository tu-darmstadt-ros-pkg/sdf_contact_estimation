#ifndef SDF_CONTACT_ESTIMATION_SHAPE_COLLISION_TYPES_H
#define SDF_CONTACT_ESTIMATION_SHAPE_COLLISION_TYPES_H

#include <std_msgs/ColorRGBA.h>

namespace sdf_contact_estimation {

  enum CollisionType {
    DEFAULT,
    TRACK,
    BODY
  };

  struct SamplingInfo {
    double resolution;
    double cylinder_angle_min;
    double cylinder_angle_max;
  };

  struct CollisionInfo {
    std::string link_name;
    CollisionType type;
    SamplingInfo sampling_info;
    std::vector<int> ignore_indices;
    std::vector<int> include_indices;
  };

  CollisionType stringToCollisionType(const std::string& str);

  std_msgs::ColorRGBA collisionTypeToColor(const CollisionType& type);
}

#endif  // SDF_CONTACT_ESTIMATION_SHAPE_COLLISION_TYPES_H
