#ifndef SDF_CONTACT_ESTIMATION_SDF_CONTACT_ESTIMATION_H
#define SDF_CONTACT_ESTIMATION_SDF_CONTACT_ESTIMATION_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <Eigen/Eigen>
#include <voxblox/core/common.h>

#include <hector_pose_prediction_interface/pose_predictor.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>

namespace sdf_contact_estimation {

using namespace hector_pose_prediction_interface;

class PoseOptimizer;

struct SdfContactEstimationSettings : public hector_pose_prediction_interface::PosePredictorSettings<double> {
  SdfContactEstimationSettings(int maximum_iterations, double contact_threshold, double tip_over_threshold,
                               bool fix_xy_coordinates, double convexity_threshold)
    : PosePredictorSettings(maximum_iterations, contact_threshold, tip_over_threshold, fix_xy_coordinates, convexity_threshold),
    iteration_contact_threshold(contact_threshold)
  {}

  explicit SdfContactEstimationSettings(const hector_pose_prediction_interface::PosePredictorSettings<double>& settings)
  : PosePredictorSettings(settings), iteration_contact_threshold(settings.contact_threshold)
  {}

  bool loadParametersFromNamespace(const ros::NodeHandle& nh) {
    contact_threshold = nh.param("final_max_contact_distance", 0.05);
    iteration_contact_threshold = nh.param("iteration_max_contact_distance", contact_threshold);
    convexity_threshold = nh.param("convexity_threshold", 0.0);
    maximum_iterations = nh.param("max_iterations", 5);
    tip_over_threshold = nh.param("robot_fall_limit", M_PI/3);
    return true;
  }

  double iteration_contact_threshold;
};

class SDFContactEstimation : public hector_pose_prediction_interface::PosePredictor<double>{
public:
  // Constructing
  SDFContactEstimation(const ros::NodeHandle &nh, const ShapeModelPtr& shape_model, const SdfModelPtr& sdf_model);
  bool loadParametersFromNamespace(ros::NodeHandle &nh);

  hector_math::RobotModel<double>::Ptr robotModel() override;
  hector_math::RobotModel<double>::ConstPtr robotModel() const override;

  SdfModelPtr getSdfModel();
  SdfModelConstPtr getSdfModel() const;

  void updateSettings(const hector_pose_prediction_interface::PosePredictorSettings<double>& settings) override;
  const hector_pose_prediction_interface::PosePredictorSettings<double>& settings() const override;

  // Joint state
//  std::vector<std::size_t> getJointIndices(const std::vector<std::string>& joints) const;

  // Access shape
  const RobotShape& getRobotShape() const;

  // Debug
  void enableVisualisation(bool enabled, const std::string& world_frame="world");
private:
  double doPredictPoseAndContactInformation(
      hector_math::Pose<double>& pose, SupportPolygon<double>& support_polygon,
      ContactInformation<double>& contact_information,
      ContactInformationFlags requested_contact_information,
      const Wrench<double> &wrench) const override;
  double doPredictPoseAndSupportPolygon(
      hector_math::Pose<double>& pose,
      SupportPolygon<double>& support_polygon,
      const Wrench<double> &wrench) const override;
  double doPredictPose(hector_math::Pose<double>& pose,
                       const Wrench<double> &wrench) const override;
  bool doEstimateSupportPolygon(
      const hector_math::Pose<double>& pose,
      SupportPolygon<double>& support_polygon) const override;


  Eigen::Isometry3d doPosePredictionStep(const Eigen::Isometry3d & initial_pose, const Eigen::Isometry3d &base_to_com, bool rotation_step, const Eigen::Isometry3d& rotation_frame) const;
  bool doEstimateContactInformation(
      const hector_math::Pose<double>& pose,
      SupportPolygon<double>& support_polygon,
      ContactInformation<double>& contact_information,
      ContactInformationFlags requested_contact_information) const override;
  bool estimateContactInformationInternal(
      const Eigen::Isometry3d& pose,
      SupportPolygon<double>& support_polygon,
      double contact_threshold,
      double convexity_threshold,
      ContactInformation<double>& contact_information,
      ContactInformationFlags requested_contact_information) const;
  bool computeRotationFrame(SupportPolygon<double>& support_polygon, const Eigen::Isometry3d& world_to_com, Eigen::Isometry3d& rotation_frame) const;

  bool robotFellOver(const Eigen::Isometry3d& robot_pose) const;

  ros::NodeHandle nh_;
  SdfContactEstimationSettings settings_;
  ShapeModelPtr shape_model_;
  SdfModelPtr sdf_model_;

  std::shared_ptr<PoseOptimizer> pose_optimizer_;

  // Parameters
  // Debug
  bool stepping_;
  bool publish_visualisation_;
  std::string world_frame_;

  // Once at start
  ros::Publisher init_pose_pub_;
  ros::Publisher init_com_pub_;
  ros::Publisher init_shape_pub_;
  ros::Publisher init_robot_state_pub_;

  // Every iteration
  ros::Publisher iteration_shape_pub_;
  ros::Publisher iteration_com_pub_;
  ros::Publisher iteration_contact_points_pub_;
  ros::Publisher iteration_support_polygon_pub_;
  ros::Publisher rotation_axis_pub_;
  ros::Publisher iteration_robot_state_pub_;

  // After last iteration
  ros::Publisher result_shape_pub_;
  ros::Publisher result_com_pub_;
  ros::Publisher result_support_polygon_pub_;
  ros::Publisher result_robot_state_pub_;
};

}

#endif
