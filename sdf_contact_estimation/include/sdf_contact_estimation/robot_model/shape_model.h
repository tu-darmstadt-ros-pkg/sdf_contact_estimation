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

/// Plain, node-free configuration for a ShapeModel. All ROS I/O (parameters,
/// /robot_description[_semantic] topics) is done once by
/// loadShapeModelConfigFromNode(); the resulting struct can build any number of
/// ShapeModels without a node, which is what allows one model per worker thread.
struct ShapeModelConfig {
  std::string urdf;                        ///< robot description (required)
  std::string srdf;                        ///< semantic description (optional, "")
  std::string collision_links_config_file; ///< path to collision_links YAML (required)
  double default_resolution = 0.1;
  rclcpp::Logger logger = rclcpp::get_logger( "shape_model" ); ///< logging only
};

class ShapeModel : public hector_math::RobotModel<double>
{
public:
  /// Node-free construction from plain config.
  explicit ShapeModel( const ShapeModelConfig &config );
  /// Back-compat: reads config from the node, then delegates to the node-free path.
  explicit ShapeModel( const rclcpp::Node::SharedPtr &node );

  const RobotShape &getShape() const;
  size_t getTotalSamplingPointCount() const;

  /// Shape -> owning link association (read-only introspection; used by
  /// external tooling, e.g. the sdf_pose_prediction_gpu fixture dump).
  const std::vector<CollisionBody> &getCollisionBodies() const { return collision_bodies_; }

  /// Name of the kinematic root link of the loaded model. All collision-body /
  /// link transforms (and hence the sampling points) are expressed relative to
  /// this frame, so a predicted/seed pose is the pose OF this frame. For the demo
  /// robot this is base_link; for Athena it is base_footprint_link.
  std::string modelRootFrame() const;
  void getRobotStateVisualization( visualization_msgs::msg::MarkerArray &marker_array,
                                   const Eigen::Isometry3d &pose, const std::string &frame_id ) const;
  void getRobotShapeVisualization( visualization_msgs::msg::MarkerArray &marker_array,
                                   const Eigen::Isometry3d &pose, std::string frame_id,
                                   const Eigen::Vector3d &color ) const;
  moveit_msgs::msg::DisplayRobotState
  getDisplayRobotStateMsg( const Eigen::Isometry3d &robot_pose ) const;
  const double &mass() const override;

  /// Block until a latched std_msgs/String arrives on `topic` (or timeout).
  /// Used by loadShapeModelConfigFromNode() to fetch the URDF/SRDF.
  static std::optional<std::string>
  waitForStringMessage( const rclcpp::Node::SharedPtr &node, const std::string &topic,
                        std::chrono::milliseconds timeout = std::chrono::seconds( 5 ) );

protected:
  hector_math::Vector3<double> computeCenterOfMass() const override;
  hector_math::Polygon<double> computeFootprint() const override;
  Eigen::AlignedBox<double, 3> computeAxisAlignedBoundingBox() const override;

  void onJointStatesUpdated() override;

private:
  void applyConfig( const ShapeModelConfig &config );
  void buildRobotModel( const std::string &urdf, const std::string &srdf );
  void generateShape();
  void updateShape();

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
  rclcpp::Logger logger_;
};

typedef std::shared_ptr<ShapeModel> ShapeModelPtr;

/// Read a ShapeModelConfig from a node: the two parameters
/// (default_resolution, collision_links_config_file) and the URDF/SRDF from the
/// /robot_description[_semantic] topics. This is the ONLY place that touches ROS.
ShapeModelConfig loadShapeModelConfigFromNode( const rclcpp::Node::SharedPtr &node );

} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_SHAPE_MODEL_H
