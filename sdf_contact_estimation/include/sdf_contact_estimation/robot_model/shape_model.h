#ifndef SDF_CONTACT_ESTIMATION_SHAPE_MODEL_H
#define SDF_CONTACT_ESTIMATION_SHAPE_MODEL_H

#include <rclcpp/rclcpp.hpp>

#include <geometric_shapes/shapes.h>
#include <hector_math/robot/robot_model.h>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>
#include <sdf_contact_estimation/robot_model/shape_collision_types.h>

namespace sdf_contact_estimation
{

struct CollisionBody {
  const moveit::core::LinkModel *link_model_ptr;
  std::size_t index;
  ShapePtr shape_ptr;
  Eigen::Isometry3d offset;
};

class ShapeModel : public hector_math::RobotModel<double>
{
public:
  explicit ShapeModel( const rclcpp::Node::SharedPtr &node );

  const RobotShape &getShape() const;
  size_t getTotalSamplingPointCount() const;
  void getRobotStateVisualization( visualization_msgs::msg::MarkerArray &marker_array,
                                   const Eigen::Isometry3d &pose, const std::string &frame_id ) const;
  void getRobotShapeVisualization( visualization_msgs::msg::MarkerArray &marker_array,
                                   const Eigen::Isometry3d &pose, std::string frame_id,
                                   const Eigen::Vector3d &color ) const;
  moveit_msgs::msg::DisplayRobotState
  getDisplayRobotStateMsg( const Eigen::Isometry3d &robot_pose ) const;
  const double &mass() const override;

protected:
  hector_math::Vector3<double> computeCenterOfMass() const override;
  hector_math::Polygon<double> computeFootprint() const override;
  Eigen::AlignedBox<double, 3> computeAxisAlignedBoundingBox() const override;

  void onJointStatesUpdated() override;

private:
  void loadParameters( const rclcpp::Node::SharedPtr node );
  void loadRobotModel( const rclcpp::Node::SharedPtr node );
  void generateShape();
  void updateShape();
  std::optional<std::string>
  waitForStringMessage( const rclcpp::Node::SharedPtr &node, const std::string &topic,
                        std::chrono::milliseconds timeout = std::chrono::seconds( 5 ) );

  static ShapePtr convertShape( const shapes::ShapeConstPtr &shape_ptr, CollisionType type,
                                SamplingInfo sampling_info );
  static ShapePtr convertBox( const shapes::Box *box, const SamplingInfo &sampling_info );
  static ShapePtr convertCylinder( const shapes::Cylinder *cylinder,
                                   const SamplingInfo &sampling_info );

  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  RobotShape shape_;
  size_t total_sampling_point_count_;
  std::vector<CollisionInfo> collision_links_;
  std::vector<CollisionBody> collision_bodies_;
  mutable double robot_mass_;

  moveit::core::JointModel *world_virtual_joint_;
  rclcpp::Node::SharedPtr node_;
};

typedef std::shared_ptr<ShapeModel> ShapeModelPtr;

} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_SHAPE_MODEL_H
