// Tests the terrain surface sampler on ESDFs with known heights and unknown
// regions.

#include <cmath>
#include <functional>
#include <limits>

#include <gtest/gtest.h>

#include <sdf_contact_estimation/sdf/terrain_surface.h>

#include "test_helpers.h"

using namespace sdf_contact_estimation;
using test::VoxelState;

namespace
{

constexpr double kFloor = 0.12;
constexpr double kTolerance = 0.005;
constexpr double kInf = std::numeric_limits<double>::infinity();

Eigen::Isometry3d pose( double x, double y, double z, double yaw )
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d( x, y, z );
  p.linear() = Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
  return p;
}

// Sets each voxel's distance to distance(voxel centre), keeping what is observed.
std::shared_ptr<voxblox::EsdfMap>
withDistances( const std::shared_ptr<voxblox::EsdfMap> &esdf,
               const std::function<double( const voxblox::Point & )> &distance )
{
  voxblox::Layer<voxblox::EsdfVoxel> *layer = esdf->getEsdfLayerPtr();
  voxblox::BlockIndexList blocks;
  layer->getAllAllocatedBlocks( &blocks );
  for ( const voxblox::BlockIndex &index : blocks ) {
    voxblox::Block<voxblox::EsdfVoxel> &block = layer->getBlockByIndex( index );
    for ( size_t linear = 0; linear < block.num_voxels(); ++linear )
      block.getVoxelByLinearIndex( linear ).distance =
          static_cast<float>( distance( block.computeCoordinatesFromLinearIndex( linear ) ) );
  }
  return esdf;
}

} // namespace

TEST( TerrainSurface, FindsThePlaneFromAboveAndBelow )
{
  const SdfModelPtr sdf = test::makeSdfModel( test::makeFloorEsdf( kFloor ) );
  EXPECT_NEAR( surfaceHeight( *sdf, 0.3, -0.2, 0.5, 0.5 ), kFloor, kTolerance );
  EXPECT_NEAR( surfaceHeight( *sdf, 0.3, -0.2, -0.2, 0.5 ), kFloor, kTolerance );
  EXPECT_EQ( surfaceHeight( *sdf, 0.3, -0.2, 0.5, 0.3 ), -kInf );
  EXPECT_EQ( surfaceHeight( *sdf, 0.3, -0.2, -0.2, 0.3 ), kInf );
  EXPECT_THROW( surfaceHeight( SdfModel(), 0.0, 0.0, 0.0, 0.5 ), std::invalid_argument );
  EXPECT_THROW( surfaceHeight( *sdf, 0.3, -0.2, 0.5, 0.0 ), std::invalid_argument );
}

TEST( TerrainSurface, FindsThinSolidUnderAnOverestimatedDistance )
{
  // One voxel of solid at z = 0.125 whose distances above overstate the gap by
  // 15 %, as a propagated ESDF does: a march by the distance steps over it.
  const SdfModelPtr sdf =
      test::makeSdfModel( withDistances( test::makeFloorEsdf(), []( const voxblox::Point &c ) {
        return 1.15 * std::abs( c.z() - 0.125 ) - 0.01;
      } ) );
  const double height = surfaceHeight( *sdf, 0.0, 0.0, 0.4, 0.5 );
  EXPECT_GT( height, 0.125 );
  EXPECT_LT( height, 0.175 );
}

TEST( TerrainSurface, UnknownColumnsAreGaps )
{
  const SdfModelPtr sdf = test::makeSdfModel( test::makeFloorEsdf( kFloor, 1.6, []( double x, double ) {
    return x > 0.8 ? VoxelState::NoBlock : x > 0.3 ? VoxelState::Unobserved : VoxelState::Observed;
  } ) );
  EXPECT_NEAR( surfaceHeight( *sdf, 0.0, 0.0, 0.4, 0.5 ), kFloor, kTolerance );
  // Next to observed voxels the extrapolation still has a crossing.
  EXPECT_TRUE( std::isnan( surfaceHeight( *sdf, 0.31, 0.0, 0.4, 0.5 ) ) );
  EXPECT_TRUE( std::isnan( surfaceHeight( *sdf, 0.5, 0.0, 0.4, 0.5 ) ) );
  EXPECT_TRUE( std::isnan( surfaceHeight( *sdf, 1.2, 0.0, 0.4, 0.5 ) ) );
}

TEST( TerrainSurface, BandFollowsThePathAcrossTheHeading )
{
  const SdfModelPtr sdf = test::makeSdfModel( test::makeFloorEsdf( kFloor ) );
  // Facing +y: the left axis points to -x.
  const SurfaceBand band =
      surfaceBand( *sdf, { pose( 0.0, -0.5, 0.4, M_PI / 2 ), pose( 0.0, 0.5, 0.4, M_PI / 2 ) },
                   { { 0.1, 0.2 } }, 0.05, 0.5 );
  // 21 rows, two triangles per cell between them.
  ASSERT_EQ( band.triangles.size(), 20u * 2 * 3 );
  EXPECT_TRUE( band.no_surface.empty() );
  EXPECT_TRUE( band.triangles[0].isApprox( Eigen::Vector3d( -0.1, -0.5, kFloor ), 0.02 ) );
  EXPECT_TRUE( band.triangles[1].isApprox( Eigen::Vector3d( -0.1, -0.45, kFloor ), 0.02 ) );
  EXPECT_TRUE( band.triangles[2].isApprox( Eigen::Vector3d( -0.2, -0.45, kFloor ), 0.02 ) );
  for ( const Eigen::Vector3d &p : band.triangles ) EXPECT_NEAR( p.z(), kFloor, kTolerance );
  EXPECT_NEAR( band.triangles[band.triangles.size() - 2].y(), 0.5, 1e-9 );
  EXPECT_THROW( surfaceBand( *sdf, { pose( 0, 0, 0, 0 ) }, { { 0.0 } }, 0.0, 0.5 ),
                std::invalid_argument );
}

TEST( TerrainSurface, UnknownLeavesGapsAndADeepDropIsMarked )
{
  // Floor at kFloor up to x = 0.3, a known drop to -0.6 up to x = 0.8, unknown beyond.
  const SdfModelPtr sdf = test::makeSdfModel( withDistances(
      test::makeFloorEsdf( kFloor, 1.6,
                           []( double x, double ) {
                             return x > 0.8 ? VoxelState::Unobserved : VoxelState::Observed;
                           } ),
      []( const voxblox::Point &c ) { return c.z() - ( c.x() < 0.3 ? kFloor : -0.6 ); } ) );
  const SurfaceBand band =
      surfaceBand( *sdf, { pose( -0.5, 0.0, 0.4, 0.0 ), pose( 1.5, 0.0, 0.4, 0.0 ) },
                   { { -0.1, 0.0, 0.1 } }, 0.05, 0.5 );
  ASSERT_FALSE( band.triangles.empty() );
  ASSERT_EQ( band.triangles.size() % 6, 0u );
  double x_max = -1.0;
  for ( const Eigen::Vector3d &p : band.triangles ) {
    EXPECT_NEAR( p.z(), kFloor, kTolerance );
    x_max = std::max( x_max, p.x() );
  }
  EXPECT_LT( x_max, 0.31 );
  EXPECT_GT( x_max, 0.2 );
  ASSERT_FALSE( band.no_surface.empty() );
  for ( const Eigen::Vector3d &p : band.no_surface ) {
    EXPECT_NEAR( p.z(), 0.4 - 0.5, 1e-9 );
    EXPECT_GT( p.x(), 0.25 );
    EXPECT_LT( p.x(), 0.8 );
  }
}

TEST( TerrainSurface, TrackOffsetsSpanEachSide )
{
  const auto tracks = trackOffsets(
      { { 0.0, 0.2, 0.0 }, { 0.5, 0.3, 0.0 }, { 0.0, -0.2, 0.0 }, { -0.4, -0.3, 0.0 } }, 3 );
  ASSERT_EQ( tracks.size(), 2u );
  ASSERT_EQ( tracks[0].size(), 3u );
  EXPECT_NEAR( tracks[0][0], 0.2, 1e-12 );
  EXPECT_NEAR( tracks[0][1], 0.25, 1e-12 );
  EXPECT_NEAR( tracks[0][2], 0.3, 1e-12 );
  ASSERT_EQ( tracks[1].size(), 3u );
  EXPECT_NEAR( tracks[1][0], -0.2, 1e-12 );
  EXPECT_NEAR( tracks[1][2], -0.3, 1e-12 );
  EXPECT_TRUE( trackOffsets( { { 0.0, 0.2, 0.0 } }, 3 )[1].empty() );
}
