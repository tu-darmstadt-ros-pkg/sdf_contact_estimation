#ifndef SDF_CONTACT_ESTIMATION_SHAPE_MODEL_H
#define SDF_CONTACT_ESTIMATION_SHAPE_MODEL_H

#include <ros/ros.h>

#include <hector_math/robot/robot_model.h>
#include <geometric_shapes/shapes.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <sdf_contact_estimation/robot_model/basic_shapes/shape_base.h>
#include <sdf_contact_estimation/robot_model/shape_collision_types.h>

namespace sdf_contact_estimation {

struct CollisionBody {
  const moveit::core::LinkModel* link_model_ptr;
  std::size_t index;
  ShapePtr shape_ptr;
  Eigen::Isometry3d offset;
};

class ShapeModel : public hector_math::RobotModel<double>{
public:
  explicit ShapeModel(const ros::NodeHandle& pnh);

  const RobotShape& getShape() const;
  size_t getTotalSamplingPointCount() const;
  void getRobotStateVisualization(visualization_msgs::MarkerArray& marker_array, const Eigen::Isometry3d& pose, const std::string& frame_id) const;
  void getRobotShapeVisualization(visualization_msgs::MarkerArray& marker_array, const Eigen::Isometry3d& pose,
                                  std::string frame_id, const Eigen::Vector3d& color) const;
  moveit_msgs::DisplayRobotState getDisplayRobotStateMsg(const Eigen::Isometry3d& robot_pose) const;
  const double& mass() const override;

protected:
  hector_math::Vector3<double> computeCenterOfMass() const override;
  hector_math::Polygon<double> computeFootprint() const override;
  Eigen::AlignedBox<double, 3> computeAxisAlignedBoundingBox() const override;

  void onJointStatesUpdated() override;
private:
  void loadParameters(const ros::NodeHandle& nh);
  void loadRobotModel(const ros::NodeHandle& nh);
  void generateShape();
  void updateShape();

  static ShapePtr convertShape(const shapes::ShapeConstPtr& shape_ptr, CollisionType type, SamplingInfo sampling_info) ;
  static ShapePtr convertBox(const shapes::Box *box, const SamplingInfo& sampling_info) ;
  static ShapePtr convertCylinder(const shapes::Cylinder *cylinder, const SamplingInfo& sampling_info) ;

  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;

  RobotShape shape_;
  size_t total_sampling_point_count_;
  std::vector<CollisionInfo> collision_links_;
  std::vector<CollisionBody> collision_bodies_;
  mutable double robot_mass_;

  moveit::core::JointModel* world_virtual_joint_;
};

typedef std::shared_ptr<ShapeModel> ShapeModelPtr;

}



#endif  // SDF_CONTACT_ESTIMATION_SHAPE_MODEL_H
