#include <sdf_contact_estimation/robot_model/shape_collision_types.h>

namespace sdf_contact_estimation
{
CollisionType stringToCollisionType( const std::string &str )
{
  if ( str == "track" ) {
    return TRACK;
  }
  if ( str == "main_track" ) {
    return MAIN_TRACK;
  }
  if ( str == "body" ) {
    return BODY;
  }
  if ( str == "default" ) {
    return DEFAULT;
  }
  return DEFAULT;
}

std_msgs::msg::ColorRGBA collisionTypeToColor( const CollisionType &type )
{
  std_msgs::msg::ColorRGBA color;
  color.a = 1;
  switch ( type ) {
  case TRACK:
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    return color;
  case MAIN_TRACK:
    color.r = 0.0;
    color.g = 0.4;
    color.b = 1.0;
    return color;
  case BODY:
    color.r = 0.8;
    color.g = 0.0;
    color.b = 0.0;
    return color;
  case DEFAULT:
    color.r = 0.0;
    color.g = 1.0;
    color.b = 1.0;
    return color;
  }
  return color;
}
} // namespace sdf_contact_estimation
