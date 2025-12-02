#ifndef SDF_CONTACT_ESTIMATION_SDF_CONTACT_ESTIMATION_H
#define SDF_CONTACT_ESTIMATION_SDF_CONTACT_ESTIMATION_H

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <voxblox/core/common.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <hector_pose_prediction_interface/pose_predictor.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>

namespace sdf_contact_estimation
{

using namespace hector_pose_prediction_interface;

class PoseOptimizer;

struct SdfContactEstimationSettings
    : public hector_pose_prediction_interface::PosePredictorSettings<double> {
  SdfContactEstimationSettings( int maximum_iterations, double contact_threshold,
                                double tip_over_threshold, bool fix_xy_coordinates,
                                double convexity_threshold )
      : PosePredictorSettings( maximum_iterations, contact_threshold, tip_over_threshold,
                               fix_xy_coordinates, convexity_threshold ),
        iteration_contact_threshold( contact_threshold ),
        chassis_contact_threshold( contact_threshold )
  {
  }

  explicit SdfContactEstimationSettings(
      const hector_pose_prediction_interface::PosePredictorSettings<double> &settings )
      : PosePredictorSettings( settings ), iteration_contact_threshold( settings.contact_threshold ),
        chassis_contact_threshold( settings.contact_threshold )
  {
  }

  bool loadParametersFromNamespace( const rclcpp::Node::SharedPtr &node )
  {
    contact_threshold = node->declare_parameter<double>( "final_contact_threshold", 0.05 );
    iteration_contact_threshold =
        node->declare_parameter<double>( "iteration_contact_threshold", contact_threshold );
    chassis_contact_threshold =
        node->declare_parameter<double>( "chassis_contact_threshold", contact_threshold );
    convexity_threshold = node->declare_parameter<double>( "convexity_threshold", 0.0 );
    maximum_iterations = node->declare_parameter<int>( "max_iterations", 5 );
    tip_over_threshold = node->declare_parameter<double>( "robot_fall_limit", M_PI / 3.0 );
    return true;
  }

  double iteration_contact_threshold;
  double chassis_contact_threshold;
};

class SDFContactEstimation : public hector_pose_prediction_interface::PosePredictor<double>
{
public:
  // Constructing
  SDFContactEstimation( const rclcpp::Node::SharedPtr node, const ShapeModelPtr &shape_model,
                        const SdfModelPtr &sdf_model );

  bool loadParametersFromNamespace( const rclcpp::Node::SharedPtr &node );

  hector_math::RobotModel<double>::Ptr robotModel() override;
  hector_math::RobotModel<double>::ConstPtr robotModel() const override;

  SdfModelPtr getSdfModel();
  SdfModelConstPtr getSdfModel() const;

  void updateSettings(
      const hector_pose_prediction_interface::PosePredictorSettings<double> &settings ) override;
  const hector_pose_prediction_interface::PosePredictorSettings<double> &settings() const override;

  // Access shape
  const RobotShape &getRobotShape() const;

  // Debug
  void enableVisualisation( bool enabled, const std::string &world_frame = "world" );

private:
  double doPredictPoseAndContactInformation( hector_math::Pose<double> &pose,
                                             SupportPolygon<double> &support_polygon,
                                             ContactInformation<double> &contact_information,
                                             ContactInformationFlags requested_contact_information,
                                             const Wrench<double> &wrench ) const override;

  double doPredictPoseAndSupportPolygon( hector_math::Pose<double> &pose,
                                         SupportPolygon<double> &support_polygon,
                                         const Wrench<double> &wrench ) const override;

  double doPredictPose( hector_math::Pose<double> &pose, const Wrench<double> &wrench ) const override;

  bool doEstimateSupportPolygon( const hector_math::Pose<double> &pose,
                                 SupportPolygon<double> &support_polygon ) const override;

  Eigen::Isometry3d doPosePredictionStep( const Eigen::Isometry3d &initial_pose,
                                          const Eigen::Isometry3d &base_to_com, bool rotation_step,
                                          const Eigen::Isometry3d &rotation_frame ) const;

  bool doEstimateContactInformation(
      const hector_math::Pose<double> &pose, SupportPolygon<double> &support_polygon,
      ContactInformation<double> &contact_information,
      ContactInformationFlags requested_contact_information ) const override;

  bool estimateContactInformationInternal(
      const Eigen::Isometry3d &pose, SupportPolygon<double> &support_polygon,
      double contact_threshold, double contact_threshold_body, double convexity_threshold,
      ContactInformation<double> &contact_information,
      ContactInformationFlags requested_contact_information ) const;

  bool computeRotationFrame( SupportPolygon<double> &support_polygon,
                             const Eigen::Isometry3d &world_to_com,
                             Eigen::Isometry3d &rotation_frame ) const;

  bool robotFellOver( const Eigen::Isometry3d &robot_pose ) const;

  rclcpp::Node::SharedPtr node_;
  SdfContactEstimationSettings settings_;
  ShapeModelPtr shape_model_;
  SdfModelPtr sdf_model_;

  std::shared_ptr<PoseOptimizer> pose_optimizer_;

  // Parameters / debug
  bool stepping_{ false };
  bool publish_visualisation_{ false };
  std::string world_frame_{ "world" };

  // Once at start
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr init_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr init_com_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr init_shape_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr init_robot_state_pub_;

  // Every iteration
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr iteration_shape_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr iteration_com_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr iteration_contact_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr iteration_support_polygon_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rotation_axis_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr iteration_robot_state_pub_;

  // After last iteration
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr result_shape_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr result_com_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr result_support_polygon_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr result_robot_state_pub_;
};

} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_SDF_CONTACT_ESTIMATION_H
