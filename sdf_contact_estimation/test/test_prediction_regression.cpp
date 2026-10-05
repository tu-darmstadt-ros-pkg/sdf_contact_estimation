// Prints the predicted poses and stabilities for fixed maps and seed poses with
// full precision. To check that a change keeps the predictions, run it before
// and after the change with SDF_CE_REGRESSION_OUT set and diff the two files.
// It uses only the public prediction API, so it also runs on older versions.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hector_math/types/pose.h>
#include <sdf_contact_estimation/test_scenarios.h>

#include "test_helpers.h"

using namespace sdf_contact_estimation;
using namespace sdf_contact_estimation::test;

namespace
{

struct Seed {
  double x, y, z, roll, pitch, yaw;
};

Eigen::Isometry3d seedPose( const Seed &s )
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d( s.x, s.y, s.z );
  pose.linear() = ( Eigen::AngleAxisd( s.yaw, Eigen::Vector3d::UnitZ() ) *
                    Eigen::AngleAxisd( s.pitch, Eigen::Vector3d::UnitY() ) *
                    Eigen::AngleAxisd( s.roll, Eigen::Vector3d::UnitX() ) )
                      .toRotationMatrix();
  return pose;
}

void dump( FILE *out, const std::string &map_name, const SdfModelPtr &sdf_model,
           const std::vector<Seed> &seeds )
{
  auto shape_model = makeTestRobot();
  SDFContactEstimation estimator( makeSettings(), shape_model, sdf_model );
  for ( size_t i = 0; i < seeds.size(); ++i ) {
    hector_math::Pose<double> pose( seedPose( seeds[i] ) );
    hector_pose_prediction_interface::SupportPolygon<double> support_polygon;
    hector_pose_prediction_interface::ContactInformation<double> contacts;
    const auto flags = static_cast<hector_pose_prediction_interface::ContactInformationFlags>(
        hector_pose_prediction_interface::contact_information_flags::LinkType |
        hector_pose_prediction_interface::contact_information_flags::Point );
    const double stability =
        estimator.predictPoseAndContactInformation( pose, support_polygon, contacts, flags );
    const Eigen::Isometry3d result = pose.asTransform();
    const Eigen::Quaterniond q( result.linear() );
    std::fprintf( out, "%s %zu %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %zu %zu\n",
                  map_name.c_str(), i, stability, result.translation().x(),
                  result.translation().y(), result.translation().z(), q.x(), q.y(), q.z(), q.w(),
                  support_polygon.contact_hull_points.size(), contacts.contact_points.size() );
  }
}

} // namespace

TEST( PredictionRegression, Dump )
{
  const char *out_path = std::getenv( "SDF_CE_REGRESSION_OUT" );
  FILE *out = out_path ? std::fopen( out_path, "w" ) : stdout;
  ASSERT_NE( out, nullptr );

  std::vector<Seed> seeds;
  for ( double x : { -1.0, -0.6, -0.3, 0.0, 0.25, 0.5, 0.8, 1.1 } ) {
    seeds.push_back( { x, 0.0, 0.4, 0.0, 0.0, 0.0 } );
    seeds.push_back( { x, 0.07, 0.5, 0.1, -0.15, 0.6 } );
    seeds.push_back( { x, -0.1, 0.3, -0.2, 0.25, 1.57 } );
  }

  // Programmatic ESDF maps.
  dump( out, "esdf_flat", makeSdfModel( makeFloorEsdf() ), seeds );
  dump( out, "esdf_holed",
        makeSdfModel( makeFloorEsdf( 0.0, 1.6,
                                     []( double x, double y ) {
                                       return ( x > -0.45 && x < 0.05 && y > 0.05 && y < 0.4 )
                                                  ? VoxelState::Unobserved
                                                  : VoxelState::Observed;
                                     } ) ),
        seeds );

  // Built-in scenarios, through both TSDF and ESDF.
  for ( const std::string scenario : { "flat", "ramp", "step_0.18", "obstacle", "hole" } ) {
    for ( bool use_esdf : { true, false } ) {
      // The TSDF integration is not bit-reproducible between runs, so the maps
      // can be cached in SDF_CE_REGRESSION_MAPS to compare predictions exactly.
      auto sdf_model = std::make_shared<SdfModel>();
      const char *map_dir = std::getenv( "SDF_CE_REGRESSION_MAPS" );
      const std::string map_file = map_dir ? std::string( map_dir ) + "/" + scenario +
                                                 ( use_esdf ? ".esdf" : ".tsdf" )
                                           : std::string();
      const bool cached = !map_file.empty() && std::ifstream( map_file ).good();
      if ( cached && use_esdf ) {
        ASSERT_TRUE( sdf_model->loadEsdfFromFile( map_file, 0.4f, false ) );
      } else if ( cached ) {
        ASSERT_TRUE( sdf_model->loadTsdfFromFile( map_file, 0.4f, false, false ) );
      } else {
        sdf_model->loadCloud( createScenarioFromName( scenario ), 0.4f, 0.05f, use_esdf );
        if ( !map_file.empty() ) {
          if ( use_esdf )
            sdf_model->saveEsdfToFile( map_file );
          else
            sdf_model->saveTsdfToFile( map_file );
        }
      }
      dump( out, scenario + ( use_esdf ? "_esdf" : "_tsdf" ), sdf_model, seeds );
    }
  }

  if ( out != stdout )
    std::fclose( out );
}
