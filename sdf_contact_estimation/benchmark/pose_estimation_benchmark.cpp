#define EIGEN_RUNTIME_NO_MALLOC

#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/util/utils.h>

#include <benchmark/benchmark.h>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

constexpr std::size_t num_configurations = 2000;

using sdf_contact_estimation::SdfModel;
using sdf_contact_estimation::ShapeModel;

static void BM_PoseEstimation( benchmark::State &state, const rclcpp::Node::SharedPtr node,
                               const std::vector<double> &joint_state_vec )
{
  // --- Parameters -----------------------------------------------------------
  bool compute_contact_information = false;
  node->get_parameter_or( "compute_contact_information", compute_contact_information, false );

  // default_state.* → map<string, double>
  std::map<std::string, double> default_state;
  {
    auto res = node->list_parameters( { "default_state" }, 10 );
    const std::string prefix = "default_state.";
    for ( const auto &name : res.names ) {
      if ( name.rfind( prefix, 0 ) != 0 ) {
        continue;
      }
      const std::string joint_name = name.substr( prefix.size() );
      double value = 0.0;
      node->get_parameter_or( name, value, 0.0 );
      default_state[joint_name] = value;
    }
  }

  // joints → vector<string>
  std::vector<std::string> joints;
  node->get_parameter_or( "joints", joints, std::vector<std::string>{} );

  std::unordered_map<std::string, double> default_state_umap( default_state.begin(),
                                                              default_state.end() );

  if ( joint_state_vec.size() != joints.size() ) {
    RCLCPP_ERROR( node->get_logger(),
                  "Benchmark test input size (%zu) does not match expected joint state "
                  "size (%zu).",
                  joint_state_vec.size(), joints.size() );
  }

  std::unordered_map<std::string, double> joint_update;
  for ( std::size_t j = 0; j < joints.size() && j < joint_state_vec.size(); ++j ) {
    joint_update.emplace( joints[j], joint_state_vec[j] );
  }

  // --- Robot model ----------------------------------------------------------
  auto shape_model = std::make_shared<ShapeModel>( node ); // uses same node / namespace
  shape_model->updateJointPositions( default_state_umap );

  // --- SDF ------------------------------------------------------------------
  auto sdf_map_node = node->create_sub_node( "sdf_map" );
  auto sdf_model = std::make_shared<SdfModel>( sdf_map_node );
  sdf_model->loadFromServer( sdf_map_node );

  // --- Contact estimation / pose predictor ---------------------------------
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor =
      std::make_shared<sdf_contact_estimation::SDFContactEstimation>( node, shape_model, sdf_model );

  // --- Test poses -----------------------------------------------------------
  std::default_random_engine engine( 0 ); // NOLINT(cert-msc51-cpp)
  std::uniform_real_distribution<double> pose_x_distribution( 0.0, 2.4 );
  std::uniform_real_distribution<double> pose_z_distribution( -0.1, 0.5 );

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> robot_poses;
  robot_poses.reserve( num_configurations );

  for ( std::size_t i = 0; i < num_configurations; ++i ) {
    Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
    // Only translation is randomized here, rotation is zero
    robot_pose.translation() =
        Eigen::Vector3d( pose_x_distribution( engine ), 0.0, pose_z_distribution( engine ) );
    robot_poses.push_back( robot_pose );
  }

  // --- Benchmark loop -------------------------------------------------------
  double failed_estimations = 0.0;
  std::size_t index = 0;

  for ( auto _ : state ) {
    // This code gets timed
    pose_predictor->robotModel()->updateJointPositions( joint_update );

    hector_math::Pose<double> robot_pose( robot_poses[index] );
    double stability;

    if ( !compute_contact_information ) {
      stability = pose_predictor->predictPose( robot_pose );
    } else {
      hector_pose_prediction_interface::SupportPolygon<double> support_polygon;
      hector_pose_prediction_interface::ContactInformation<double> contact_information;
      stability = pose_predictor->predictPoseAndContactInformation( robot_pose, support_polygon,
                                                                    contact_information );
    }

    if ( std::isnan( stability ) ) {
      ++failed_estimations;
    }

    if ( ++index == robot_poses.size() ) {
      index = 0;
    }
  }

  state.counters["Failed"] = failed_estimations;
  state.counters["Failed (%)"] =
      benchmark::Counter( 100.0 * failed_estimations, benchmark::Counter::kAvgIterations );
}

// Load benchmarks.* parameters into (name, joint_vector) pairs
std::vector<std::pair<std::string, std::vector<double>>>
loadBenchmarks( const rclcpp::Node::SharedPtr &node )
{
  std::vector<std::pair<std::string, std::vector<double>>> test_inputs;

  auto res = node->list_parameters( { "benchmarks" }, 10 );
  if ( res.names.empty() ) {
    RCLCPP_WARN( node->get_logger(), "No 'benchmarks.*' parameters found; no benchmarks will be "
                                     "registered." );
    return test_inputs;
  }

  const std::string prefix = "benchmarks.";

  for ( const auto &full_name : res.names ) {
    if ( full_name.rfind( prefix, 0 ) != 0 ) {
      continue;
    }

    const std::string benchmark_name = full_name.substr( prefix.size() );
    std::vector<double> joint_positions;
    node->get_parameter_or( full_name, joint_positions, std::vector<double>{} );

    test_inputs.emplace_back( benchmark_name, joint_positions );
  }

  return test_inputs;
}

int main( int argc, char **argv )
{
  // Init ROS 2
  rclcpp::init( argc, argv );
  auto node = std::make_shared<rclcpp::Node>( "pose_estimation_benchmark" );

  // Declare basic parameters (so get_parameter_or works cleanly)
  node->declare_parameter<bool>( "compute_contact_information", false );
  node->declare_parameter<std::vector<std::string>>( "joints", std::vector<std::string>{} );

  // Load benchmark configurations from parameters
  auto test_inputs = loadBenchmarks( node );

  for ( const auto &test : test_inputs ) {
    benchmark::RegisterBenchmark( test.first.c_str(), BM_PoseEstimation, node, test.second )
        ->Unit( benchmark::kMillisecond )
        ->Iterations( static_cast<int>( num_configurations * 2.0 ) );
  }

  // Google Benchmark does not like extra ROS arguments: strip them
  int argc2 = 1;
  char *argv2[1]{ argv[0] };
  benchmark::Initialize( &argc2, argv2 );
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();

  rclcpp::shutdown();
  return 0;
}
