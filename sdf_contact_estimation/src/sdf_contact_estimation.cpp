#include <sdf_contact_estimation/sdf_contact_estimation.h>

#include <ctime>
#include <iostream>
#include <limits>
#include <sstream>

#include <boost/filesystem.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <voxblox/core/tsdf_map.h>

#include <hector_pose_prediction_ros/visualization.h>
#include <hector_stability_metrics/math/support_polygon.h>

#include <sdf_contact_estimation/optimization/pose_optimizer.h>
#include <sdf_contact_estimation/sdf/sdf_query_scope.h>
#include <sdf_contact_estimation/util/timing.h>
#include <sdf_contact_estimation/util/utils.h>
#include <sdf_contact_estimation/visualisation.h>

// fallback for timing macros
#ifndef START_TIMING
  #define START_TIMING( name )                                                                     \
    do {                                                                                           \
    } while ( 0 )
#endif

#ifndef STOP_TIMING_AVG
  #define STOP_TIMING_AVG                                                                          \
    do {                                                                                           \
    } while ( 0 )
#endif

#ifndef INIT_TIMING
  #define STOP_TIMING_AVG                                                                          \
    do {                                                                                           \
    } while ( 0 )
#endif

INIT_TIMING

namespace sdf_contact_estimation
{

using namespace hector_pose_prediction_interface;

SDFContactEstimation::SDFContactEstimation( const rclcpp::Node::SharedPtr node,
                                            const ShapeModelPtr &shape_model,
                                            const SdfModelPtr &sdf_model )
    : PosePredictor(), node_( node ), logger_( node->get_logger() ),
      settings_( 5, 0.05, M_PI / 3.0, false, 0.0 ), shape_model_( shape_model ),
      sdf_model_( sdf_model ), publish_visualisation_( false )
{
  loadParametersFromNamespace( node_ );
  init();
}

SDFContactEstimation::SDFContactEstimation( const SdfContactEstimationSettings &settings,
                                            const ShapeModelPtr &shape_model,
                                            const SdfModelPtr &sdf_model, rclcpp::Logger logger )
    : PosePredictor(), node_( nullptr ), logger_( std::move( logger ) ), settings_( settings ),
      shape_model_( shape_model ), sdf_model_( sdf_model ), publish_visualisation_( false )
{
  init();
}

void SDFContactEstimation::init()
{
  pose_optimizer_ = std::make_shared<PoseOptimizer>( logger_, *sdf_model_, shape_model_,
                                                     settings_.iteration_contact_threshold );
}

bool SDFContactEstimation::loadParametersFromNamespace( const rclcpp::Node::SharedPtr &node )
{
  // Debug flag (we don't change global logger level here; just read it)
  bool debug = node->declare_parameter<bool>( "debug", false );
  if ( debug ) {
    RCLCPP_INFO( logger_, "SDFContactEstimation: debug logging enabled (parameter)" );
  }

  stepping_ = node->declare_parameter<bool>( "stepping", false );

  settings_.loadParametersFromNamespace( node_ );
  return true;
}

const RobotShape &SDFContactEstimation::getRobotShape() const { return shape_model_->getShape(); }

Eigen::Isometry3d
SDFContactEstimation::doPosePredictionStep( const Eigen::Isometry3d &initial_pose,
                                            const Eigen::Isometry3d &base_to_com, bool rotation_step,
                                            const Eigen::Isometry3d &rotation_frame ) const
{
  Eigen::Isometry3d initial_com_pose = initial_pose * base_to_com;
  Eigen::Isometry3d result_com_pose;
  if ( !rotation_step ) {
    result_com_pose = pose_optimizer_->doFallingStep( initial_com_pose, base_to_com );
  } else {
    result_com_pose =
        pose_optimizer_->doRotationStep( initial_com_pose, base_to_com, rotation_frame );
  }

  Eigen::Isometry3d result_pose = result_com_pose * base_to_com.inverse();

  if ( publish_visualisation_ ) {
    visualization_msgs::msg::MarkerArray robot_state_marker_array;
    shape_model_->getRobotShapeVisualization( robot_state_marker_array, result_pose, world_frame_,
                                              Eigen::Vector3d( 0.25, 0.95, 0.57 ) );
    shape_model_->getRobotStateVisualization( robot_state_marker_array, result_pose, world_frame_ );
    if ( iteration_shape_pub_ ) {
      iteration_shape_pub_->publish( robot_state_marker_array );
    }
    publishPoint( iteration_com_pub_, result_com_pose.translation(), world_frame_ );
    if ( iteration_robot_state_pub_ ) {
      iteration_robot_state_pub_->publish( shape_model_->getDisplayRobotStateMsg( result_pose ) );
    }
  }
  return result_pose;
}

void SDFContactEstimation::enableVisualisation( bool enabled, const std::string &world_frame )
{
  if ( enabled == publish_visualisation_ ) {
    return;
  }
  if ( enabled && !node_ ) {
    RCLCPP_WARN( logger_,
                 "enableVisualisation(true) requires a node; ignoring (node-free instance)." );
    return;
  }

  if ( enabled ) {
    auto qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ).transient_local();

    // Start up publishers
    init_pose_pub_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>( "init/robot_pose", qos );
    init_com_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>( "init/com", qos );
    init_shape_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>( "init/robot_shape", qos );
    init_robot_state_pub_ =
        node_->create_publisher<moveit_msgs::msg::DisplayRobotState>( "init/robot_state", qos );

    iteration_com_pub_ =
        node_->create_publisher<geometry_msgs::msg::PointStamped>( "iteration/com", qos );
    iteration_shape_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>( "iteration/robot_shape", qos );
    iteration_contact_points_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "iteration/contact_points", qos );
    iteration_support_polygon_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "iteration/support_polygon", qos );
    rotation_axis_pub_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>( "iteration/rotation_axis", qos );
    iteration_robot_state_pub_ =
        node_->create_publisher<moveit_msgs::msg::DisplayRobotState>( "iteration/robot_state", qos );

    result_shape_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>( "result/robot_shape", qos );
    result_com_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>( "result/com", qos );
    result_support_polygon_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "result/support_polygon", qos );
    result_robot_state_pub_ =
        node_->create_publisher<moveit_msgs::msg::DisplayRobotState>( "result/robot_state", qos );

    world_frame_ = world_frame;
    sdf_model_->setWorldFrame( world_frame );
  } else {
    // Drop the publishers
    init_pose_pub_.reset();
    init_com_pub_.reset();
    init_shape_pub_.reset();
    init_robot_state_pub_.reset();

    iteration_com_pub_.reset();
    iteration_shape_pub_.reset();
    iteration_contact_points_pub_.reset();
    iteration_support_polygon_pub_.reset();
    rotation_axis_pub_.reset();
    iteration_robot_state_pub_.reset();

    result_shape_pub_.reset();
    result_com_pub_.reset();
    result_support_polygon_pub_.reset();
    result_robot_state_pub_.reset();
  }

  publish_visualisation_ = enabled;
}

bool SDFContactEstimation::robotFellOver( const Eigen::Isometry3d &robot_pose ) const
{
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d up_world = robot_pose.linear() * up;
  double inclination_angle = std::acos( up.dot( up_world ) );
  return inclination_angle > settings_.tip_over_threshold;
}

hector_math::RobotModel<double>::Ptr SDFContactEstimation::robotModel()
{
  return std::static_pointer_cast<hector_math::RobotModel<double>>( shape_model_ );
}

hector_math::RobotModel<double>::ConstPtr SDFContactEstimation::robotModel() const
{
  return std::static_pointer_cast<hector_math::RobotModel<double>>( shape_model_ );
}

void SDFContactEstimation::updateSettings(
    const hector_pose_prediction_interface::PosePredictorSettings<double> &settings )
{
  settings_ = SdfContactEstimationSettings( settings );
  init();
}

const hector_pose_prediction_interface::PosePredictorSettings<double> &
SDFContactEstimation::settings() const
{
  return settings_;
}

SdfModelPtr SDFContactEstimation::getSdfModel() { return sdf_model_; }

SdfModelConstPtr SDFContactEstimation::getSdfModel() const { return sdf_model_; }

double SDFContactEstimation::doPredictPoseAndContactInformation(
    hector_math::Pose<double> &pose, SupportPolygon<double> &support_polygon,
    ContactInformation<double> &contact_information,
    ContactInformationFlags requested_contact_information, const Wrench<double> & /*wrench*/ ) const
{
  START_TIMING( "SDFContactEstimation::doPredictPoseAndContactInformation" )
  // One prediction is one span of queries on an unchanging map.
  const SdfQueryScope sdf_query_scope;

  // Filled in along the way, published to last_diagnostics_ at every exit.
  PredictionDiagnostics diagnostics;
  auto finish = [&]( PredictionStatus status ) {
    diagnostics.status = status;
    last_diagnostics_ = diagnostics;
  };

  Eigen::Isometry3d pose_eigen = pose.asTransform();
  Eigen::Isometry3d base_to_com( Eigen::Isometry3d::Identity() );
  base_to_com.translation() = shape_model_->centerOfMass();

  // Debug: Write out start pose and COM
  Eigen::Vector3d pose_rpy = rotToNormalizedRpy( pose_eigen.linear() );
  const Eigen::Vector3d &pose_xyz = pose_eigen.translation();
  RCLCPP_DEBUG(
      logger_, "[SDFContactEstimation::estimateSupportPolygon] Estimation for pose: [%f, %f, %f, %f, %f, %f]",
      pose_xyz( 0 ), pose_xyz( 1 ), pose_xyz( 2 ), pose_rpy( 0 ), pose_rpy( 1 ), pose_rpy( 2 ) );

  const Eigen::Vector3d &com_xyz = base_to_com.translation();
  RCLCPP_DEBUG( logger_, "[SDFContactEstimation::estimateSupportPolygon] COM: [%f, %f, %f]",
                com_xyz( 0 ), com_xyz( 1 ), com_xyz( 2 ) );

  // Debug: Publish start pose of robot
  if ( publish_visualisation_ ) {
    publishPose( init_pose_pub_, pose_eigen, world_frame_ );
    Eigen::Isometry3d com_pose = pose_eigen * base_to_com;
    publishPoint( init_com_pub_, com_pose.translation(), world_frame_ );

    visualization_msgs::msg::MarkerArray robot_state_marker_array;
    shape_model_->getRobotShapeVisualization( robot_state_marker_array, pose_eigen, world_frame_,
                                              Eigen::Vector3d( 1, 0, 0 ) );
    shape_model_->getRobotStateVisualization( robot_state_marker_array, pose_eigen, world_frame_ );

    deleteAllMarkers( init_shape_pub_ );
    if ( init_shape_pub_ ) {
      init_shape_pub_->publish( robot_state_marker_array );
    }
    if ( init_robot_state_pub_ ) {
      init_robot_state_pub_->publish( shape_model_->getDisplayRobotStateMsg( pose_eigen ) );
    }
  }

  // Check if SDF is set
  if ( !sdf_model_->isLoaded() ) {
    RCLCPP_ERROR_STREAM( logger_, "No Sdf set." );
    finish( PredictionStatus::NoSdf );
    STOP_TIMING_AVG
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Start iterative contact estimation
  int iteration_counter = 0;
  bool stable = false;
  Eigen::Isometry3d next_rotation_frame;

  do {
    RCLCPP_DEBUG_STREAM( logger_, " --- Iteration " << iteration_counter << " --- " );

    // Estimate next pose
    bool rotation_step = ( iteration_counter != 0 );
    const Eigen::Isometry3d previous_pose = pose_eigen;
    pose_eigen = doPosePredictionStep( pose_eigen, base_to_com, rotation_step, next_rotation_frame );
    diagnostics.iterations = iteration_counter + 1;
    diagnostics.last_step_translation =
        ( pose_eigen.translation() - previous_pose.translation() ).norm();
    diagnostics.last_step_rotation =
        Eigen::AngleAxisd( previous_pose.linear().transpose() * pose_eigen.linear() ).angle();

    // Check if robot fell over
    if ( robotFellOver( pose_eigen ) ) {
      RCLCPP_DEBUG( logger_, "Robot fell over, stopping estimation" );
      pose = hector_math::Pose<double>( pose_eigen );
      // No contact estimation runs at the fallen pose; count it separately (failure path only).
      countUnknownSamplingPoints( pose_eigen, diagnostics );
      finish( PredictionStatus::FellOver );
      STOP_TIMING_AVG
      return -std::numeric_limits<double>::max();
    }

    // Estimate resulting contact points and next rotation_step axis
    estimateContactInformationInternal( pose_eigen, support_polygon,
                                        settings_.iteration_contact_threshold,
                                        settings_.iteration_contact_threshold, 0.0,
                                        contact_information, ContactInformationFlags::None,
                                        &diagnostics );

    // Check for valid solution
    if ( support_polygon.contact_hull_points.empty() ) {
      RCLCPP_DEBUG( logger_, "No convex hull points after iteration %d. Pose prediction failed.",
                    iteration_counter );
      pose = hector_math::Pose<double>( pose_eigen );
      finish( PredictionStatus::NoHullIteration );
      STOP_TIMING_AVG
      return std::numeric_limits<double>::quiet_NaN();
    }

    Eigen::Isometry3d world_to_com_tf = pose_eigen * base_to_com;
    stable = computeRotationFrame( support_polygon, world_to_com_tf, next_rotation_frame );

    if ( publish_visualisation_ ) {
      visualization_msgs::msg::MarkerArray support_polygon_marker_array;
      deleteAllMarkers( iteration_support_polygon_pub_ );
      visualization::addSupportPolygonToMarkerArray( support_polygon_marker_array, support_polygon,
                                                     world_frame_ );
      if ( iteration_support_polygon_pub_ ) {
        iteration_support_polygon_pub_->publish( support_polygon_marker_array );
      }
    }

    // Create new request for next iteration
    iteration_counter++;

    // Wait for input if in stepping mode
    if ( stepping_ ) {
      std::cout << '\n' << "Press ENTER to continue...";
      while ( rclcpp::ok() && std::cin.get() != '\n' ) {
        // wait
      }
    }
  } while ( !stable && iteration_counter < settings_.maximum_iterations );

  pose = hector_math::Pose<double>( pose_eigen );

  // Check for valid solution
  if ( !stable ) {
    RCLCPP_DEBUG( logger_,
                  "Robot is not stable (or fell over) after %d iterations. Pose prediction failed",
                  iteration_counter );
    finish( PredictionStatus::NotConverged );
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Estimate again with higher threshold
  estimateContactInformationInternal(
      pose_eigen, support_polygon, settings_.contact_threshold, settings_.chassis_contact_threshold,
      settings_.convexity_threshold, contact_information, requested_contact_information,
      &diagnostics );

  if ( support_polygon.contact_hull_points.empty() ) {
    RCLCPP_DEBUG( logger_, "No convex hull points in contact prediction with higher threshold. "
                           "This should not happen. Pose prediction failed." );
    pose = hector_math::Pose<double>( pose_eigen );
    finish( PredictionStatus::NoHullFinal );
    STOP_TIMING_AVG
    return std::numeric_limits<double>::quiet_NaN();
  }

  Eigen::Isometry3d world_to_com_tf = pose_eigen * base_to_com;
  support_polygon.edge_stabilities = computeForceAngleStabilitiesWithGravity<double>(
      support_polygon.contact_hull_points, world_to_com_tf.translation() );
  auto min = std::min_element( begin( support_polygon.edge_stabilities ),
                               end( support_polygon.edge_stabilities ) );

  if ( publish_visualisation_ ) {
    visualization_msgs::msg::MarkerArray robot_state_marker_array;
    shape_model_->getRobotShapeVisualization( robot_state_marker_array, pose_eigen, world_frame_,
                                              Eigen::Vector3d( 0.25, 0.95, 0.57 ) );
    shape_model_->getRobotStateVisualization( robot_state_marker_array, pose_eigen, world_frame_ );
    deleteAllMarkers( result_shape_pub_ );
    if ( result_shape_pub_ ) {
      result_shape_pub_->publish( robot_state_marker_array );
    }

    Eigen::Isometry3d com_pose = pose_eigen * base_to_com;
    publishPoint( result_com_pub_, com_pose.translation(), world_frame_ );
    if ( result_robot_state_pub_ ) {
      result_robot_state_pub_->publish( shape_model_->getDisplayRobotStateMsg( pose_eigen ) );
    }

    visualization_msgs::msg::MarkerArray support_polygon_marker_array;
    deleteAllMarkers( result_support_polygon_pub_ );
    visualization::addSupportPolygonToMarkerArray( support_polygon_marker_array, support_polygon,
                                                   world_frame_ );
    if ( result_support_polygon_pub_ ) {
      result_support_polygon_pub_->publish( support_polygon_marker_array );
    }
  }

  finish( PredictionStatus::Ok );
  STOP_TIMING_AVG

  return *min;
}

double SDFContactEstimation::doPredictPoseAndSupportPolygon( hector_math::Pose<double> &pose,
                                                             SupportPolygon<double> &support_polygon,
                                                             const Wrench<double> &wrench ) const
{
  ContactInformation<double> contact_information;
  ContactInformationFlags flags = ContactInformationFlags::None;
  return doPredictPoseAndContactInformation( pose, support_polygon, contact_information, flags,
                                             wrench );
}

double SDFContactEstimation::doPredictPose( hector_math::Pose<double> &pose,
                                            const Wrench<double> &wrench ) const
{
  SupportPolygon<double> support_polygon;
  return doPredictPoseAndSupportPolygon( pose, support_polygon, wrench );
}

bool SDFContactEstimation::doEstimateSupportPolygon( const hector_math::Pose<double> &pose,
                                                     SupportPolygon<double> &support_polygon ) const
{
  ContactInformation<double> contact_information;
  ContactInformationFlags flags = ContactInformationFlags::None;
  return doEstimateContactInformation( pose, support_polygon, contact_information, flags );
}

bool SDFContactEstimation::doEstimateContactInformation(
    const hector_math::Pose<double> &pose, SupportPolygon<double> &support_polygon,
    ContactInformation<double> &contact_information,
    ContactInformationFlags requested_contact_information ) const
{
  const SdfQueryScope sdf_query_scope;
  // Evaluates the given pose, no prediction: see lastDiagnostics().
  PredictionDiagnostics diagnostics;
  diagnostics.status = PredictionStatus::NotPredicted;
  const bool result = estimateContactInformationInternal(
      pose.asTransform(), support_polygon, settings_.contact_threshold,
      settings_.chassis_contact_threshold, settings_.convexity_threshold, contact_information,
      requested_contact_information, &diagnostics );
  last_diagnostics_ = diagnostics;
  return result;
}

void SDFContactEstimation::countUnknownSamplingPoints( const Eigen::Isometry3d &pose,
                                                       PredictionDiagnostics &diagnostics ) const
{
  int sampling_points = 0;
  int unknown_sampling_points = 0;
  for ( const ShapePtr &shape : shape_model_->getShape() ) {
    for ( const Eigen::Vector3d &p : shape->getSamplingPoints() ) {
      const Eigen::Vector3d p_world = pose * p;
      bool touched_unknown = false;
      sdf_model_->getSdf<double>( p_world.x(), p_world.y(), p_world.z(), &touched_unknown );
      ++sampling_points;
      unknown_sampling_points += touched_unknown ? 1 : 0;
    }
  }
  diagnostics.sampling_points = sampling_points;
  diagnostics.unknown_sampling_points = unknown_sampling_points;
}

bool SDFContactEstimation::estimateContactInformationInternal(
    const Eigen::Isometry3d &pose, SupportPolygon<double> &support_polygon,
    double contact_threshold, double contact_threshold_body, double convexity_threshold,
    ContactInformation<double> &contact_information,
    ContactInformationFlags requested_contact_information, PredictionDiagnostics *diagnostics ) const
{
  START_TIMING( "SDFContactEstimation::estimateContactInformationInternal" )
  int sampling_point_count = 0;
  int unknown_sampling_point_count = 0;

  if ( requested_contact_information > ContactInformationFlags::None ) {
    contact_information.contact_points.reserve( shape_model_->getTotalSamplingPointCount() );
  }

  // Find all contact points by checking their sdf value
  std::vector<Eigen::Vector3d> contact_points;
  contact_points.reserve( shape_model_->getTotalSamplingPointCount() );

  for ( const ShapePtr &shape : shape_model_->getShape() ) {
    const std::vector<Eigen::Vector3d> &sampling_points = shape->getSamplingPoints();
    for ( const Eigen::Vector3d &p : sampling_points ) {
      // Transform to world
      Eigen::Vector3d p_world = pose * p;

      // check for contact
      double sdf;
      Eigen::Vector3d contact_normal;
      bool touched_unknown = false;
      if ( requested_contact_information & ContactInformationFlags::SurfaceNormal ) {
        sdf = sdf_model_->getDistanceAndGradient( p_world, contact_normal, &touched_unknown );
      } else {
        sdf = std::abs( sdf_model_->getSdf<double>( p_world( 0 ), p_world( 1 ), p_world( 2 ),
                                                    &touched_unknown ) );
      }
      ++sampling_point_count;
      unknown_sampling_point_count += touched_unknown ? 1 : 0;

      double threshold = shape->isBody() ? contact_threshold_body : contact_threshold;
      if ( sdf < threshold ) {
        contact_points.push_back( p_world );

        if ( requested_contact_information > ContactInformationFlags::None ) {
          ContactPointInformation<double> contact_point_information;
          if ( shape->isTrack() ) {
            contact_point_information.link_type = LinkType::Tracks;
          } else if ( shape->isBody() ) {
            contact_point_information.link_type = LinkType::Chassis;
          } else {
            contact_point_information.link_type = LinkType::Undefined;
          }
          contact_point_information.point = p_world;
          contact_point_information.surface_area =
              shape->getSamplingResolution() * shape->getSamplingResolution();
          contact_point_information.surface_normal = contact_normal;
          contact_information.contact_points.push_back( contact_point_information );
        }
      }
    }
  }

  if ( diagnostics != nullptr ) {
    diagnostics->sampling_points = sampling_point_count;
    diagnostics->unknown_sampling_points = unknown_sampling_point_count;
  }

  RCLCPP_DEBUG_STREAM( logger_, "Number of contacts: " << contact_points.size()
                                                       << " with threshold " << contact_threshold );

  // Compute convex hull
  support_polygon.contact_hull_points =
      hector_stability_metrics::math::supportPolygonFromUnsortedContactPoints( contact_points );
  if ( convexity_threshold > 0.0 ) {
    support_polygon.contact_hull_points =
        supportPolygonAngleFilter( support_polygon.contact_hull_points, convexity_threshold );
  }
  support_polygon.edge_stabilities.clear();

  // Debug publishers
  if ( publish_visualisation_ ) {
    deleteAllMarkers( iteration_contact_points_pub_ );
    visualization_msgs::msg::MarkerArray contacts_marker_array;
    visualization::addContactPointsToMarkerArray( contacts_marker_array, contact_points,
                                                  Eigen::Vector4f( 1.0f, 0.5f, 0, 1.0f ),
                                                  world_frame_, "candidates", 0.03 );
    visualization::addContactPointsToMarkerArray(
        contacts_marker_array, support_polygon.contact_hull_points,
        Eigen::Vector4f( 0, 1.0f, 0, 1.0f ), world_frame_, "convex_hull", 0.04 );

    if ( iteration_contact_points_pub_ ) {
      iteration_contact_points_pub_->publish( contacts_marker_array );
    }
  }

  STOP_TIMING_AVG
  return true;
}

bool SDFContactEstimation::computeRotationFrame( SupportPolygon<double> &support_polygon,
                                                 const Eigen::Isometry3d &world_to_com,
                                                 Eigen::Isometry3d &rotation_frame ) const
{
  // 4 cases: no contact points (failure), one contact point, contact line, contact polygon
  if ( support_polygon.contact_hull_points.empty() ) {
    RCLCPP_DEBUG( logger_,
                  "[sdf_contact_estimation::computeRotationFrame] No contact points with ground." );
    return false;
  }

  Eigen::Vector3d rotation_axis;

  if ( support_polygon.contact_hull_points.size() == 1 ) {
    rotation_frame.translation() = support_polygon.contact_hull_points[0];
    Eigen::Vector3d vec_t_com = world_to_com.translation() - support_polygon.contact_hull_points[0];
    Eigen::Vector3d rotation_plane_normal = vec_t_com.cross( Eigen::Vector3d( 0, 0, -1 ) );
    rotation_axis = rotation_plane_normal;
  } else {
    if ( support_polygon.contact_hull_points.size() == 2 ) {
      RCLCPP_DEBUG( logger_, "Detected line contact" );
      rotation_axis = support_polygon.contact_hull_points[1] - support_polygon.contact_hull_points[0];
      rotation_frame.translation() = support_polygon.contact_hull_points[0];
    } else {
      RCLCPP_DEBUG( logger_, "Detected polygon contact." );
      support_polygon.edge_stabilities = computeForceAngleStabilitiesWithGravity<double>(
          support_polygon.contact_hull_points, world_to_com.translation() );
      auto min = std::min_element( begin( support_polygon.edge_stabilities ),
                                   end( support_polygon.edge_stabilities ) );
      RCLCPP_DEBUG_STREAM( logger_, "Stability: " << *min );
      if ( *min > 0 ) {
        // Stable
        return true;
      }
      unsigned int min_idx =
          static_cast<unsigned int>( min - begin( support_polygon.edge_stabilities ) );
      rotation_frame.translation() = support_polygon.contact_hull_points[min_idx];
      unsigned int min_idx_p1 = ( min_idx + 1 ) % support_polygon.contact_hull_points.size();
      rotation_axis = support_polygon.contact_hull_points[min_idx_p1] -
                      support_polygon.contact_hull_points[min_idx];
    }
  }

  RCLCPP_DEBUG( logger_, "Next rotation axis: %f, %f, %f", rotation_axis.x(), rotation_axis.y(),
                rotation_axis.z() );

  rotation_frame.linear() =
      computeGravityAlignedRotationFromTo( Eigen::Vector3d::UnitX(), rotation_axis );

  RCLCPP_DEBUG_STREAM( logger_, "Next rotation frame: " << rotation_frame.linear() );

  publishPose( rotation_axis_pub_, rotation_frame, world_frame_ );

  // Not stable yet
  return false;
}

} // namespace sdf_contact_estimation
