#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/util/timing.h>
#include <sdf_contact_estimation/util/utils.h>
#include <sdf_contact_estimation/visualisation.h>
#include <sdf_contact_estimation/optimization/pose_optimizer.h>

#include <ctime>
#include <boost/filesystem.hpp>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <voxblox/core/tsdf_map.h>

#include <hector_stability_metrics/math/support_polygon.h>
#include <hector_pose_prediction_ros/visualization.h>

INIT_TIMING

namespace sdf_contact_estimation {

using namespace hector_pose_prediction_interface;

SDFContactEstimation::SDFContactEstimation(const ros::NodeHandle& nh,
                                           const ShapeModelPtr& shape_model,
                                           const SdfModelPtr& sdf_model)
  : PosePredictor(), nh_(nh), settings_(5, 0.05, M_PI/3, false, 0.0), shape_model_(shape_model), sdf_model_(sdf_model), publish_visualisation_(false)
{
  loadParametersFromNamespace(nh_);

  pose_optimizer_ = std::make_shared<PoseOptimizer>(nh_, *sdf_model_, shape_model_, settings_.iteration_contact_threshold);
}

bool SDFContactEstimation::loadParametersFromNamespace(ros::NodeHandle &nh)
{
  if (nh.param("debug", false)) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  stepping_ = nh.param("stepping", false);
  settings_.loadParametersFromNamespace(nh_);
  return true;
}

const RobotShape& SDFContactEstimation::getRobotShape() const
{
  return shape_model_->getShape();
}

Eigen::Isometry3d SDFContactEstimation::doPosePredictionStep(const Eigen::Isometry3d & initial_pose, const Eigen::Isometry3d &base_to_com, bool rotation_step, const Eigen::Isometry3d & rotation_frame) const
{
  Eigen::Isometry3d initial_com_pose = initial_pose * base_to_com;
  Eigen::Isometry3d result_com_pose;
  if (!rotation_step) {
    result_com_pose = pose_optimizer_->doFallingStep(initial_com_pose, base_to_com);
  } else {
    result_com_pose = pose_optimizer_->doRotationStep(initial_com_pose, base_to_com, rotation_frame);
  }

  Eigen::Isometry3d result_pose = result_com_pose * base_to_com.inverse();

  if (publish_visualisation_) {
    visualization_msgs::MarkerArray robot_state_marker_array;
    shape_model_->getRobotShapeVisualization(robot_state_marker_array, result_pose, world_frame_, Eigen::Vector3d(0.25, 0.95, 0.57));
    shape_model_->getRobotStateVisualization(robot_state_marker_array, result_pose, world_frame_);
    iteration_shape_pub_.publish(robot_state_marker_array);
    publishPoint(iteration_com_pub_, result_com_pose.translation(), world_frame_);
    iteration_robot_state_pub_.publish(shape_model_->getDisplayRobotStateMsg(result_pose));
  }
  return result_pose;
}

void SDFContactEstimation::enableVisualisation(bool enabled, const std::string& world_frame)
{
  if (enabled == publish_visualisation_) {
    return;
  }
  if (enabled) {
    // Start up publishers
    init_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("init/robot_pose", 10, true);
    init_com_pub_ = nh_.advertise<geometry_msgs::PointStamped >("init/com", 10, true);
    init_shape_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("init/robot_shape", 10, true);
    init_robot_state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("init/robot_state", 10, true);

    iteration_com_pub_ = nh_.advertise<geometry_msgs::PointStamped>("iteration/com", 10, true);
    iteration_shape_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("iteration/robot_shape", 10, true);
    iteration_contact_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("iteration/contact_points", 10, true);
    iteration_support_polygon_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("iteration/support_polygon", 10, true);
    rotation_axis_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("iteration/rotation_axis", 10, true);
    iteration_robot_state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("iteration/robot_state", 10, true);


    result_shape_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("result/robot_shape", 10, true);
    result_com_pub_ = nh_.advertise<geometry_msgs::PointStamped>("result/com", 10, true);
    result_support_polygon_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("result/support_polygon", 10, true);
    result_robot_state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("result/robot_state", 10, true);
    world_frame_ = world_frame;
    sdf_model_->setWorldFrame(world_frame);
  } else {
    init_pose_pub_.shutdown();
    init_com_pub_.shutdown();
    init_shape_pub_.shutdown();
    init_robot_state_pub_.shutdown();
    iteration_com_pub_.shutdown();
    iteration_shape_pub_.shutdown();
    iteration_robot_state_pub_.shutdown();
    iteration_contact_points_pub_.shutdown();
    iteration_support_polygon_pub_.shutdown();
    rotation_axis_pub_.shutdown();
    result_shape_pub_.shutdown();
    result_com_pub_.shutdown();
    result_support_polygon_pub_.shutdown();
    result_robot_state_pub_.shutdown();
  }
  publish_visualisation_ = enabled;
}

//std::vector<std::size_t> SDFContactEstimation::getJointIndices(const std::vector<std::string>& joints) const
//{
//  std::vector<std::size_t> indices;
//  const std::vector<std::string>& joint_names = getJointNames();
//  for (const std::string& joint: joints) {
//    std::size_t idx = static_cast<std::size_t>(std::find(joint_names.begin(), joint_names.end(), joint) - joint_names.begin());
//    if (idx >= joint_names.size()) {
//      ROS_ERROR_STREAM("Joint '" << joint << "' wasn't found in contact estimation model.");
//    }
//    indices.push_back(idx);
//  }
//  return indices;
//}

bool SDFContactEstimation::robotFellOver(const Eigen::Isometry3d& robot_pose) const
{
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d up_world = robot_pose.linear() * up; // third column of rotation matrix
  double inclination_angle = std::acos(up.dot(up_world));
  return inclination_angle > settings_.tip_over_threshold;
}

hector_math::RobotModel<double>::Ptr SDFContactEstimation::robotModel()
{
  return std::static_pointer_cast<hector_math::RobotModel<double>>(shape_model_);
}
hector_math::RobotModel<double>::ConstPtr SDFContactEstimation::robotModel() const
{
  return std::static_pointer_cast<hector_math::RobotModel<double>>(shape_model_);
}
void SDFContactEstimation::updateSettings(
    const hector_pose_prediction_interface::PosePredictorSettings<double>& settings)
{
  settings_ = SdfContactEstimationSettings(settings);
  pose_optimizer_ = std::make_shared<PoseOptimizer>(nh_, *sdf_model_, shape_model_, settings_.iteration_contact_threshold);
}
const hector_pose_prediction_interface::PosePredictorSettings<double>& SDFContactEstimation::settings() const
{
  return settings_;
}

SdfModelPtr SDFContactEstimation::getSdfModel() {
  return sdf_model_;
}

SdfModelConstPtr SDFContactEstimation::getSdfModel() const {
  return sdf_model_;
}

double SDFContactEstimation::doPredictPoseAndContactInformation(
    hector_math::Pose<double>& pose, SupportPolygon<double>& support_polygon,
    ContactInformation<double>& contact_information,
    ContactInformationFlags requested_contact_information,
    const Wrench<double> &wrench) const
{
  START_TIMING("SDFContactEstimation::doPredictPoseAndContactInformation")
  Eigen::Isometry3d pose_eigen = pose.asTransform();
  Eigen::Isometry3d base_to_com(Eigen::Isometry3d::Identity());
  base_to_com.translation() = shape_model_->centerOfMass();

  // Debug: Write out start pose and COM
  Eigen::Vector3d pose_rpy = rotToNormalizedRpy(pose_eigen.linear());
  const Eigen::Vector3d& pose_xyz = pose_eigen.translation();
  ROS_DEBUG("[SDFContactEstimation::estimateSupportPolygon] Estimation for pose: [%f, %f, %f, %f, %f, %f]",
            pose_xyz(0), pose_xyz(1), pose_xyz(2), pose_rpy(0), pose_rpy(1), pose_rpy(2));
  const Eigen::Vector3d& com_xyz = base_to_com.translation();
  ROS_DEBUG("[SDFContactEstimation::estimateSupportPolygon] COM: [%f, %f, %f]", com_xyz(0), com_xyz(1), com_xyz(2));

  // Debug: Publish start pose of robot
  if (publish_visualisation_) {
    publishPose(init_pose_pub_, pose_eigen, world_frame_);
    Eigen::Isometry3d com_pose = pose_eigen * base_to_com;
    publishPoint(init_com_pub_, com_pose.translation(), world_frame_);
    visualization_msgs::MarkerArray robot_state_marker_array;
    shape_model_->getRobotShapeVisualization(robot_state_marker_array, pose_eigen, world_frame_, Eigen::Vector3d(1, 0, 0));
    shape_model_->getRobotStateVisualization(robot_state_marker_array, pose_eigen, world_frame_);
    deleteAllMarkers(init_shape_pub_);
    init_shape_pub_.publish(robot_state_marker_array);
    init_robot_state_pub_.publish(shape_model_->getDisplayRobotStateMsg(pose_eigen));
  }

  // Check if SDF is set
  if (!sdf_model_->isLoaded()) {
    ROS_ERROR_STREAM("No Sdf set.");
    STOP_TIMING_AVG
    return std::nan("");
  }

  // Start iterative contact estimation
  int iteration_counter = 0;
  bool stable = false;
  Eigen::Isometry3d next_rotation_frame;
  do {
    ROS_DEBUG_STREAM(" --- Iteration " << iteration_counter << " --- ");

    // Estimate next pose
    bool rotation_step = (iteration_counter != 0);
    pose_eigen =
        doPosePredictionStep(pose_eigen, base_to_com, rotation_step, next_rotation_frame);

    // Check if robot fell over
    if (robotFellOver(pose_eigen)) {
      ROS_DEBUG_STREAM("Robot fell over, stopping estimation");
      pose = hector_math::Pose<double>(pose_eigen);
      STOP_TIMING_AVG
      return -std::numeric_limits<double>::max();
    }

    // Estimate resulting contact points and next rotation_step axis
    estimateContactInformationInternal(pose_eigen, support_polygon, settings_.iteration_contact_threshold, settings_.iteration_contact_threshold, 0.0, contact_information, ContactInformationFlags::None);
    // Check for valid solution
    if (support_polygon.contact_hull_points.empty()) {
      ROS_DEBUG_STREAM("No convex hull points after iteration " << iteration_counter << ". Pose prediction failed.");
      pose = hector_math::Pose<double>(pose_eigen);
      STOP_TIMING_AVG
      return std::nan("");
    }
    Eigen::Isometry3d world_to_com_tf = pose_eigen * base_to_com;
    stable = computeRotationFrame(support_polygon, world_to_com_tf, next_rotation_frame);

    if (publish_visualisation_) {
      visualization_msgs::MarkerArray support_polygon_marker_array;
      deleteAllMarkers(iteration_support_polygon_pub_);
      visualization::addSupportPolygonToMarkerArray(support_polygon_marker_array, support_polygon, world_frame_);
      iteration_support_polygon_pub_.publish(support_polygon_marker_array);
    }

    // Create new request for next iteration
    iteration_counter++;

    // Wait for input if in stepping mode
    if (stepping_) {
      std::cout << '\n' << "Press ENTER to continue...";
      do {
      } while (ros::ok() && std::cin.get() != '\n');
    }
  } while (!stable && iteration_counter < settings_.maximum_iterations); // Stop if stable or max iterations exceeded

  pose = hector_math::Pose<double>(pose_eigen);

  // Check for valid solution
  if (!stable) {
    ROS_DEBUG_STREAM("Robot is not stable (or fell over) after " << iteration_counter << " iterations. Pose prediction failed");
    return std::nan("");
  }

  // Estimate again with higher threshold
  estimateContactInformationInternal(pose_eigen, support_polygon, settings_.contact_threshold, settings_.chassis_contact_threshold, settings_.convexity_threshold, contact_information, requested_contact_information);
  if (support_polygon.contact_hull_points.empty()) {
    ROS_DEBUG_STREAM("No convex hull points in contact prediction with higher threshold. This should not happen. Pose prediction failed.");
    pose = hector_math::Pose<double>(pose_eigen);
    STOP_TIMING_AVG
    return std::nan("");
  }
  Eigen::Isometry3d world_to_com_tf = pose_eigen * base_to_com;
  support_polygon.edge_stabilities = computeForceAngleStabilitiesWithGravity<double>(support_polygon.contact_hull_points, world_to_com_tf.translation());
  auto min = std::min_element(begin(support_polygon.edge_stabilities), end(support_polygon.edge_stabilities));

  if (publish_visualisation_) {
    visualization_msgs::MarkerArray robot_state_marker_array;
    shape_model_->getRobotShapeVisualization(robot_state_marker_array, pose_eigen, world_frame_, Eigen::Vector3d(0.25, 0.95, 0.57));
    shape_model_->getRobotStateVisualization(robot_state_marker_array, pose_eigen, world_frame_);
    deleteAllMarkers(result_shape_pub_);
    result_shape_pub_.publish(robot_state_marker_array);
    Eigen::Isometry3d com_pose = pose_eigen * base_to_com;
    publishPoint(result_com_pub_, com_pose.translation(), world_frame_);
    result_robot_state_pub_.publish(shape_model_->getDisplayRobotStateMsg(pose_eigen));

    visualization_msgs::MarkerArray support_polygon_marker_array;
    deleteAllMarkers(result_support_polygon_pub_);
    visualization::addSupportPolygonToMarkerArray(support_polygon_marker_array, support_polygon, world_frame_);
    result_support_polygon_pub_.publish(support_polygon_marker_array);
  }

  STOP_TIMING_AVG

  return *min;

}
double SDFContactEstimation::doPredictPoseAndSupportPolygon(
    hector_math::Pose<double>& pose,
    SupportPolygon<double>& support_polygon,
    const Wrench<double> &wrench) const
{
  ContactInformation<double> contact_information;
  ContactInformationFlags flags = ContactInformationFlags::None;
  return doPredictPoseAndContactInformation(pose, support_polygon, contact_information, flags, wrench);
}
double SDFContactEstimation::doPredictPose(hector_math::Pose<double>& pose,
                                           const Wrench<double> &wrench) const
{
  SupportPolygon<double> support_polygon;
  return doPredictPoseAndSupportPolygon(pose, support_polygon, wrench);
}
bool SDFContactEstimation::doEstimateSupportPolygon(
    const hector_math::Pose<double>& pose,
    SupportPolygon<double>& support_polygon) const
{
  ContactInformation<double> contact_information;
  ContactInformationFlags flags = ContactInformationFlags::None;
  return estimateContactInformation(pose, support_polygon, contact_information, flags);
}

bool SDFContactEstimation::doEstimateContactInformation(
    const hector_math::Pose<double>& pose,
    SupportPolygon<double>& support_polygon,
    ContactInformation<double>& contact_information,
    ContactInformationFlags requested_contact_information) const
{
  return estimateContactInformationInternal(pose.asTransform(), support_polygon, settings_.contact_threshold, settings_.chassis_contact_threshold, settings_.convexity_threshold, contact_information, requested_contact_information);
}

bool SDFContactEstimation::estimateContactInformationInternal(
    const Eigen::Isometry3d& pose,
    SupportPolygon<double>& support_polygon,
    double contact_threshold,
    double contact_threshold_body,
    double convexity_threshold,
    ContactInformation<double>& contact_information,
    ContactInformationFlags requested_contact_information) const
{
  START_TIMING("SDFContactEstimation::estimateContactInformationInternal")
  if (requested_contact_information > ContactInformationFlags::None) {
    contact_information.contact_points.reserve(shape_model_->getTotalSamplingPointCount());
  }
  // Find all contact points by checking their sdf value
  std::vector<Eigen::Vector3d> contact_points;
  contact_points.reserve(shape_model_->getTotalSamplingPointCount());
  for (const ShapePtr& shape: shape_model_->getShape()) {
    const std::vector<Eigen::Vector3d>& sampling_points = shape->getSamplingPoints();
    for (const Eigen::Vector3d& p: sampling_points) {
      // Transform to world
      Eigen::Vector3d p_world = pose * p;
      // check for contact
      double sdf;
      Eigen::Vector3d contact_normal;
      if (requested_contact_information & ContactInformationFlags::SurfaceNormal) {
        sdf = sdf_model_->getDistanceAndGradient(p_world, contact_normal);
      } else {
        sdf = std::abs(sdf_model_->getSdf<double>(p_world(0), p_world(1), p_world(2)));
      }
      double threshold = shape->isBody() ? contact_threshold_body : contact_threshold;
      if (sdf < threshold) {
        contact_points.push_back(p_world);

        if (requested_contact_information > ContactInformationFlags::None)
        {
          ContactPointInformation<double> contact_point_information;
          if (shape->isTrack())
            contact_point_information.link_type = LinkType::Tracks;
          else if (shape->isBody())
            contact_point_information.link_type = LinkType::Chassis;
          else
            contact_point_information.link_type = LinkType::Undefined;
          contact_point_information.point = p_world;
          contact_point_information.surface_area = shape->getSamplingResolution() * shape->getSamplingResolution();
          contact_point_information.surface_normal = contact_normal;
          contact_information.contact_points.push_back(contact_point_information);
        }
      }
    }
  }
  ROS_DEBUG_STREAM("Number of contacts: " << contact_points.size() << " with threshold " << contact_threshold);

  // Compute convex hull
  support_polygon.contact_hull_points = hector_stability_metrics::math::supportPolygonFromUnsortedContactPoints(contact_points);
  if (convexity_threshold > 0.0) {
    support_polygon.contact_hull_points = supportPolygonAngleFilter(support_polygon.contact_hull_points, convexity_threshold);
  }
  support_polygon.edge_stabilities.clear();

  // Debug publishers
  if (publish_visualisation_) {
    deleteAllMarkers(iteration_contact_points_pub_);
    visualization_msgs::MarkerArray contacts_marker_array;
    visualization::addContactPointsToMarkerArray(contacts_marker_array, contact_points, Eigen::Vector4f(1.0f, 0.5f, 0, 1.0f), world_frame_, "candidates", 0.03);
    visualization::addContactPointsToMarkerArray(contacts_marker_array, support_polygon.contact_hull_points, Eigen::Vector4f(0, 1.0f, 0, 1.0f), world_frame_, "convex_hull", 0.04);

    iteration_contact_points_pub_.publish(contacts_marker_array);
  }

  STOP_TIMING_AVG
  return true;
}

bool SDFContactEstimation::computeRotationFrame(SupportPolygon<double>& support_polygon, const Eigen::Isometry3d& world_to_com, Eigen::Isometry3d& rotation_frame) const
{
  // 4 cases: no contact points (failure), one contact point, contact line, contact polygon
  if (support_polygon.contact_hull_points.empty()) {
    ROS_DEBUG_STREAM("[sdf_contact_estimation::computeRotationFrame] No contact points with ground.");
    return false;
  }

  // check for point
  Eigen::Vector3d rotation_axis;
  if (support_polygon.contact_hull_points.size() == 1) {
    rotation_frame.translation() = support_polygon.contact_hull_points[0];
    Eigen::Vector3d vec_t_com = world_to_com.translation() - support_polygon.contact_hull_points[0];
    Eigen::Vector3d rotation_plane_normal = vec_t_com.cross(Eigen::Vector3d(0, 0, -1)); // Cross product between contact-com vector and gravity
    rotation_axis = rotation_plane_normal;
  } else {
    if (support_polygon.contact_hull_points.size() == 2) {
      ROS_DEBUG_STREAM("Detected line contact");
      rotation_axis = support_polygon.contact_hull_points[1] - support_polygon.contact_hull_points[0];
      rotation_frame.translation() = support_polygon.contact_hull_points[0];
    } else {
      ROS_DEBUG_STREAM("Detected polygon contact.");
      support_polygon.edge_stabilities = computeForceAngleStabilitiesWithGravity<double>(support_polygon.contact_hull_points, world_to_com.translation());
      auto min = std::min_element(begin(support_polygon.edge_stabilities), end(support_polygon.edge_stabilities));
      ROS_DEBUG_STREAM("Stability: " << *min);
      if (*min > 0) {
        // Stable
        return true;
      }
      unsigned int min_idx = min - begin(support_polygon.edge_stabilities);
      rotation_frame.translation() = support_polygon.contact_hull_points[min_idx];
      unsigned int min_idx_p1 = (min_idx+1) % support_polygon.contact_hull_points.size();
      rotation_axis = support_polygon.contact_hull_points[min_idx_p1] - support_polygon.contact_hull_points[min_idx];
    }
  }

  // compute a transform that aligns the x-axis with the rotation axis
  ROS_DEBUG_STREAM("Next rotation axis: " << rotation_axis.x() << ", " << rotation_axis.y() << ", " << rotation_axis.z());
  rotation_frame.linear() = computeGravityAlignedRotationFromTo(Eigen::Vector3d::UnitX(), rotation_axis);

  ROS_DEBUG_STREAM("Next rotation frame: " << rotation_frame.linear());
  publishPose(rotation_axis_pub_, rotation_frame, world_frame_);

  // Not stable
  return false;
}

}
