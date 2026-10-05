// Tests for PredictionDiagnostics: prediction status, step counts and the
// count of sampling points whose SDF query touched unknown space.

#include <cstdio>
#include <limits>
#include <memory>

#include <voxblox/core/tsdf_map.h>

#include <gtest/gtest.h>

#include <hector_math/types/pose.h>
#include <sdf_contact_estimation/prediction_diagnostics.h>

#include "test_helpers.h"

using namespace sdf_contact_estimation;
using namespace sdf_contact_estimation::test;
namespace hpp = hector_pose_prediction_interface;

namespace
{

struct Result {
  double stability;
  Eigen::Isometry3d pose;
  PredictionDiagnostics diagnostics;
};

Result predict( const SdfModelPtr &sdf_model, const Eigen::Isometry3d &seed,
                const char *label = nullptr )
{
  SDFContactEstimation estimator( makeSettings(), makeTestRobot(), sdf_model );
  hector_math::Pose<double> pose( seed );
  hpp::SupportPolygon<double> support_polygon;
  hpp::ContactInformation<double> contacts;
  const auto flags = static_cast<hpp::ContactInformationFlags>(
      hpp::contact_information_flags::LinkType | hpp::contact_information_flags::Point );
  Result r;
  r.stability = estimator.predictPoseAndContactInformation( pose, support_polygon, contacts, flags );
  r.pose = pose.asTransform();
  r.diagnostics = estimator.lastDiagnostics();
  if ( label != nullptr ) {
    const PredictionDiagnostics &d = r.diagnostics;
    std::printf( "[diagnostics] %-28s status=%s iterations=%d step=(%.4g m, %.4g rad) "
                 "unknown=%d/%d stability=%.4g z=%.4g\n",
                 label, toString( d.status ), d.iterations, d.last_step_translation,
                 d.last_step_rotation, d.unknown_sampling_points, d.sampling_points, r.stability,
                 r.pose.translation().z() );
  }
  return r;
}

Eigen::Isometry3d seedAt( double x, double y, double z )
{
  Eigen::Isometry3d seed = Eigen::Isometry3d::Identity();
  seed.translation() = Eigen::Vector3d( x, y, z );
  return seed;
}

// Patch under the rear half of the left track (track: x in [-0.3, 0.3], y = 0.2 +- 0.05).
VoxelState rearLeftPatch( double x, double y, VoxelState hole )
{
  return ( x > -0.45 && x < 0.0 && y > 0.05 && y < 0.4 ) ? hole : VoxelState::Observed;
}

} // namespace

TEST( PredictionDiagnostics, ToString )
{
  EXPECT_STREQ( toString( PredictionStatus::Ok ), "ok" );
  EXPECT_STREQ( toString( PredictionStatus::NoSdf ), "no_sdf" );
  EXPECT_STREQ( toString( PredictionStatus::FellOver ), "fell_over" );
  EXPECT_STREQ( toString( PredictionStatus::NoHullIteration ), "no_hull_iteration" );
  EXPECT_STREQ( toString( PredictionStatus::NotConverged ), "not_converged" );
  EXPECT_STREQ( toString( PredictionStatus::NoHullFinal ), "no_hull_final" );
  EXPECT_STREQ( toString( PredictionStatus::NotPredicted ), "not_predicted" );
  EXPECT_EQ( static_cast<int>( PredictionStatus::NotPredicted ), 6 );
}

TEST( PredictionDiagnostics, DefaultIsNotPredicted )
{
  SDFContactEstimation estimator( makeSettings(), makeTestRobot(),
                                  makeSdfModel( makeFloorEsdf() ) );
  EXPECT_EQ( estimator.lastDiagnostics().status, PredictionStatus::NotPredicted );
  EXPECT_EQ( estimator.lastDiagnostics().iterations, 0 );
}

TEST( PredictionDiagnostics, NoSdf )
{
  Result r = predict( std::make_shared<SdfModel>(), seedAt( 0, 0, 0.3 ), "no sdf" );
  EXPECT_EQ( r.diagnostics.status, PredictionStatus::NoSdf );
  EXPECT_EQ( r.diagnostics.iterations, 0 );
  EXPECT_EQ( r.diagnostics.sampling_points, 0 );
}

TEST( PredictionDiagnostics, FlatFloorIsOkWithoutUnknown )
{
  Result r = predict( makeSdfModel( makeFloorEsdf() ), seedAt( 0, 0, 0.3 ), "flat floor" );
  const size_t total = makeTestRobot()->getTotalSamplingPointCount();
  EXPECT_EQ( r.diagnostics.status, PredictionStatus::Ok );
  EXPECT_TRUE( std::isfinite( r.stability ) );
  EXPECT_NEAR( r.pose.translation().z(), 0.0, 0.03 );
  EXPECT_GE( r.diagnostics.iterations, 1 );
  EXPECT_TRUE( std::isfinite( r.diagnostics.last_step_translation ) );
  EXPECT_TRUE( std::isfinite( r.diagnostics.last_step_rotation ) );
  EXPECT_EQ( r.diagnostics.sampling_points, static_cast<int>( total ) );
  EXPECT_EQ( r.diagnostics.unknown_sampling_points, 0 );
}

TEST( PredictionDiagnostics, FellOverCountsAtTheFallenPose )
{
  // Rolled beyond the tip-over threshold (60 deg): the first step keeps the roll.
  Eigen::Isometry3d seed = seedAt( 0, 0, 0.3 );
  seed.linear() = Eigen::AngleAxisd( 1.3, Eigen::Vector3d::UnitX() ).toRotationMatrix();
  Result r = predict( makeSdfModel( makeFloorEsdf() ), seed, "fell over" );
  EXPECT_EQ( r.diagnostics.status, PredictionStatus::FellOver );
  EXPECT_EQ( r.stability, -std::numeric_limits<double>::max() );
  EXPECT_EQ( r.diagnostics.iterations, 1 );
  EXPECT_TRUE( std::isfinite( r.diagnostics.last_step_translation ) );
  EXPECT_EQ( r.diagnostics.sampling_points,
             static_cast<int>( makeTestRobot()->getTotalSamplingPointCount() ) );
}

TEST( PredictionDiagnostics, UnobservedPatchIsCounted )
{
  auto esdf = makeFloorEsdf( 0.0, 1.6, []( double x, double y ) {
    return rearLeftPatch( x, y, VoxelState::Unobserved );
  } );
  Result r = predict( makeSdfModel( esdf ), seedAt( 0, 0, 0.3 ), "unobserved patch rear-left" );
  EXPECT_GT( r.diagnostics.unknown_sampling_points, 0 );
  EXPECT_LT( r.diagnostics.unknown_sampling_points, r.diagnostics.sampling_points );
  EXPECT_NE( r.diagnostics.status, PredictionStatus::NotPredicted );
}

TEST( PredictionDiagnostics, MissingBlocksAreCounted )
{
  // Leave out every block with x < 0 and y > 0 (blocks are 0.8 m): the rear-left
  // quarter of the robot stands over unallocated space.
  auto esdf = makeFloorEsdf( 0.0, 1.6, []( double x, double y ) {
    return ( x < 0.0 && y > 0.0 ) ? VoxelState::NoBlock : VoxelState::Observed;
  } );
  Result r = predict( makeSdfModel( esdf ), seedAt( 0, 0, 0.3 ), "missing blocks rear-left" );
  EXPECT_GT( r.diagnostics.unknown_sampling_points, 0 );
  EXPECT_NE( r.diagnostics.status, PredictionStatus::NotPredicted );
}

TEST( PredictionDiagnostics, UnknownUnderWholeRobot )
{
  auto esdf = makeFloorEsdf( 0.0, 1.6, []( double x, double y ) {
    return ( std::abs( x ) < 0.6 && std::abs( y ) < 0.5 ) ? VoxelState::Unobserved
                                                          : VoxelState::Observed;
  } );
  Result r = predict( makeSdfModel( esdf ), seedAt( 0, 0, 0.3 ), "unobserved under whole robot" );
  EXPECT_NE( r.diagnostics.status, PredictionStatus::Ok );
  EXPECT_GT( r.diagnostics.unknown_sampling_points, 0 );
}

TEST( PredictionDiagnostics, EstimateAtPoseIsNotPredicted )
{
  auto esdf = makeFloorEsdf( 0.0, 1.6, []( double x, double y ) {
    return rearLeftPatch( x, y, VoxelState::Unobserved );
  } );
  SDFContactEstimation estimator( makeSettings(), makeTestRobot(), makeSdfModel( esdf ) );
  hpp::SupportPolygon<double> support_polygon;
  EXPECT_TRUE( estimator.estimateSupportPolygon(
      hector_math::Pose<double>( seedAt( 0, 0, 0.0 ) ), support_polygon ) );
  const PredictionDiagnostics &d = estimator.lastDiagnostics();
  EXPECT_EQ( d.status, PredictionStatus::NotPredicted );
  EXPECT_EQ( d.iterations, 0 );
  EXPECT_GT( d.unknown_sampling_points, 0 );
  EXPECT_EQ( d.sampling_points, static_cast<int>( makeTestRobot()->getTotalSamplingPointCount() ) );
}

TEST( PredictionDiagnostics, CountingDoesNotChangeTheSdf )
{
  auto esdf = makeFloorEsdf( 0.0, 1.6, []( double x, double y ) {
    return rearLeftPatch( x, y, VoxelState::Unobserved );
  } );
  SdfModelPtr model = makeSdfModel( esdf );
  for ( double x = -0.6; x <= 0.6; x += 0.0173 ) {
    for ( double y = -0.1; y <= 0.5; y += 0.0191 ) {
      for ( double z : { -0.02, 0.0, 0.013, 0.05 } ) {
        bool touched = true;
        const double with = model->getSdf<double>( x, y, z, &touched );
        const double without = model->getSdf<double>( x, y, z );
        EXPECT_EQ( with, without );
        // Columns well inside the patch must report unknown, well outside must not.
        if ( x > -0.35 && x < -0.1 && y > 0.15 && y < 0.3 )
          EXPECT_TRUE( touched ) << x << " " << y << " " << z;
        if ( x > 0.1 || y < -0.0 )
          EXPECT_FALSE( touched ) << x << " " << y << " " << z;
      }
    }
  }
}

TEST( PredictionDiagnostics, ScanAlongPatch )
{
  // Informational: how the status develops as the robot moves over the patch.
  auto esdf = makeFloorEsdf( 0.0, 1.6, []( double x, double y ) {
    return rearLeftPatch( x, y, VoxelState::Unobserved );
  } );
  SdfModelPtr model = makeSdfModel( esdf );
  for ( double x : { -1.0, -0.6, -0.3, 0.0, 0.25, 0.5, 0.8 } ) {
    char label[64];
    std::snprintf( label, sizeof( label ), "patch scan x=%.2f", x );
    Result r = predict( model, seedAt( x, 0, 0.3 ), label );
    EXPECT_NE( r.diagnostics.status, PredictionStatus::NotPredicted );
  }
}

TEST( PredictionDiagnostics, UnknownUnderOneTrack )
{
  // Informational: the whole left side unobserved, only the right track has ground.
  for ( VoxelState hole : { VoxelState::Unobserved, VoxelState::NoBlock } ) {
    auto esdf = makeFloorEsdf( 0.0, 1.6, [hole]( double, double y ) {
      return y > 0.05 ? hole : VoxelState::Observed;
    } );
    Result r = predict( makeSdfModel( esdf ), seedAt( 0, 0, 0.3 ),
                        hole == VoxelState::NoBlock ? "left side missing blocks"
                                                    : "left side unobserved" );
    EXPECT_NE( r.diagnostics.status, PredictionStatus::Ok );
    EXPECT_GT( r.diagnostics.unknown_sampling_points, 0 );
  }
}

namespace
{

// TSDF counterpart of makeFloorEsdf(): unobserved voxels have weight 0.
std::shared_ptr<voxblox::TsdfMap> makeFloorTsdf( const std::function<VoxelState( double, double )> &state )
{
  voxblox::TsdfMap::Config config;
  config.tsdf_voxel_size = 0.05;
  config.tsdf_voxels_per_side = 16;
  auto tsdf = std::make_shared<voxblox::TsdfMap>( config );
  voxblox::Layer<voxblox::TsdfVoxel> *layer = tsdf->getTsdfLayerPtr();
  for ( int bx = -2; bx < 2; ++bx ) {
    for ( int by = -2; by < 2; ++by ) {
      for ( int bz = -1; bz < 1; ++bz ) {
        const voxblox::Point center = voxblox::getCenterPointFromGridIndex(
            voxblox::BlockIndex( bx, by, bz ), layer->block_size() );
        if ( state( center.x(), center.y() ) == VoxelState::NoBlock )
          continue;
        auto block = layer->allocateBlockPtrByIndex( voxblox::BlockIndex( bx, by, bz ) );
        for ( size_t linear = 0; linear < block->num_voxels(); ++linear ) {
          const voxblox::Point c = block->computeCoordinatesFromLinearIndex( linear );
          voxblox::TsdfVoxel &voxel = block->getVoxelByLinearIndex( linear );
          voxel.distance = static_cast<float>( c.z() );
          voxel.weight = state( c.x(), c.y() ) == VoxelState::Observed ? 1.0f : 0.0f;
        }
      }
    }
  }
  return tsdf;
}

// Blocks are 0.8 m: x < -0.8 is a missing block, x in [-0.4, 0) unobserved.
VoxelState stripes( double x, double )
{
  if ( x < -0.8 )
    return VoxelState::NoBlock;
  return ( x > -0.4 && x < 0.0 ) ? VoxelState::Unobserved : VoxelState::Observed;
}

// Queries well inside each region, near the floor.
template<typename Query>
void expectFlagPerRegion( const Query &query, const char *name )
{
  for ( double z : { -0.02, 0.013 } ) {
    for ( double x : { -1.2, -0.2, 0.4 } ) {
      bool touched = x > 0.0; // overwritten in both directions
      query( x, 0.1, z, &touched );
      EXPECT_EQ( touched, x < 0.0 ) << name << " x=" << x << " z=" << z;
    }
  }
}

} // namespace

TEST( PredictionDiagnostics, EveryInterpolationPathSetsTheFlag )
{
  using cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxESDF;
  using cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxTSDF;
  auto esdf_map = makeFloorEsdf( 0.0, 1.6, stripes );
  auto tsdf_map = makeFloorTsdf( stripes );
  for ( bool cubic : { false, true } ) {
    for ( bool extrapolate : { false, true } ) {
      const InterpolatedVoxbloxESDF esdf( esdf_map, 0.4f, cubic, extrapolate );
      const InterpolatedVoxbloxTSDF tsdf( tsdf_map, 0.4f, cubic, extrapolate );
      const std::string config = std::string( cubic ? "cubic" : "trilinear" ) +
                                 ( extrapolate ? " extrapolated" : "" );
      expectFlagPerRegion(
          [&]( double x, double y, double z, bool *t ) { esdf.GetSDF<double>( x, y, z, 1, t ); },
          ( "esdf " + config ).c_str() );
      expectFlagPerRegion(
          [&]( double x, double y, double z, bool *t ) { tsdf.GetSDF<double>( x, y, z, 1, t ); },
          ( "tsdf " + config ).c_str() );
      if ( !cubic ) {
        expectFlagPerRegion(
            [&]( double x, double y, double z, bool *t ) {
              Eigen::Vector3d gradient;
              esdf.GetSDFAndGradient<double>( x, y, z, gradient, 1, t );
            },
            ( "esdf gradient " + config ).c_str() );
      }
    }
  }
}
