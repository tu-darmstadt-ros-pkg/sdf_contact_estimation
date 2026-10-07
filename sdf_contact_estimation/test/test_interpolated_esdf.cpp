// Tests InterpolatedVoxbloxESDF, with and without the dense grid, against
// corner lookups in the voxblox layer.

#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <random>
#include <tuple>

#include <gtest/gtest.h>

#include <voxblox/core/esdf_map.h>

#include <sdf_contact_estimation/sdf/interpolated_voxblox_esdf.h>
#include <sdf_contact_estimation/sdf/sdf_query_scope.h>

using cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxESDF;
using Data = InterpolatedVoxbloxESDF::InterpolationData;

namespace
{

constexpr float kTruncation = 0.4f;

// Blocks in [-2, 2) x [-2, 2) x [-1, 1), a quarter of them left out, one in twenty of
// the voxels unobserved, distances in [-0.5, 0.5].
std::shared_ptr<voxblox::EsdfMap> makeRandomEsdf( std::mt19937 &rng )
{
  voxblox::EsdfMap::Config config;
  config.esdf_voxel_size = 0.05;
  config.esdf_voxels_per_side = 16;
  auto esdf = std::make_shared<voxblox::EsdfMap>( config );
  voxblox::Layer<voxblox::EsdfVoxel> *layer = esdf->getEsdfLayerPtr();
  std::uniform_real_distribution<float> distance( -0.5f, 0.5f );
  std::uniform_real_distribution<double> u( 0.0, 1.0 );
  for ( int bx = -2; bx < 2; ++bx ) {
    for ( int by = -2; by < 2; ++by ) {
      for ( int bz = -1; bz < 1; ++bz ) {
        if ( u( rng ) < 0.25 )
          continue;
        auto block = layer->allocateBlockPtrByIndex( voxblox::BlockIndex( bx, by, bz ) );
        for ( size_t linear = 0; linear < block->num_voxels(); ++linear ) {
          voxblox::EsdfVoxel &voxel = block->getVoxelByLinearIndex( linear );
          voxel.distance = distance( rng );
          voxel.observed = u( rng ) > 0.05;
        }
      }
    }
  }
  return esdf;
}

// Corner data read voxel by voxel from the layer, with the interpolator's
// corner placement and unknown voxel rules (boundary extrapolation).
Data referenceData( const voxblox::Layer<voxblox::EsdfVoxel> &layer, double x, double y, double z,
                    bool *touched_unknown )
{
  const float res = layer.voxel_size();
  const float round = 1 / res;
  const float offset = 0.5 * res;
  const auto lower = [&]( double v ) {
    return static_cast<float>( ( std::floor( v * round + 0.5 ) / round ) - offset );
  };
  Data d{};
  d.x1 = lower( x );
  d.y1 = lower( y );
  d.z1 = lower( z );
  d.x2 = static_cast<float>( d.x1 ) + res;
  d.y2 = static_cast<float>( d.y1 ) + res;
  d.z2 = static_cast<float>( d.z1 ) + res;
  double *q[8] = { &d.q111, &d.q112, &d.q121, &d.q122, &d.q211, &d.q212, &d.q221, &d.q222 };
  int invalid = 0;
  double sum = 0.0;
  for ( int c = 0; c < 8; ++c ) {
    const voxblox::Point p( c & 4 ? d.x2 : d.x1, c & 2 ? d.y2 : d.y1, c & 1 ? d.z2 : d.z1 );
    const voxblox::EsdfVoxel *voxel = layer.getVoxelPtrByCoordinates( p );
    if ( voxel != nullptr && voxel->observed ) {
      *q[c] = voxel->distance;
      sum += *q[c];
    } else {
      *q[c] = NAN;
      ++invalid;
    }
  }
  *touched_unknown = invalid > 0;
  for ( int c = 0; c < 8; ++c ) {
    if ( std::isnan( *q[c] ) )
      *q[c] = sum < 0 ? -kTruncation : kTruncation;
  }
  return d;
}

void expectSameData( const Data &a, const Data &b )
{
  EXPECT_EQ( std::memcmp( &a, &b, sizeof( Data ) ), 0 )
      << "corner " << a.x1 << " " << a.y1 << " " << a.z1 << " values " << a.q111 << " " << b.q111
      << " ... " << a.q222 << " " << b.q222;
}

// Parameters: dense grid, inside an SdfQueryScope (block reuse across queries).
class InterpolatedEsdfModes : public ::testing::TestWithParam<std::tuple<bool, bool>>
{
};

} // namespace

TEST_P( InterpolatedEsdfModes, MatchesLayerLookups )
{
  const auto [dense_grid, in_scope] = GetParam();
  std::mt19937 rng( 7 );
  const auto esdf = makeRandomEsdf( rng );
  const InterpolatedVoxbloxESDF interpolator( esdf, kTruncation, false, true, dense_grid );
  const voxblox::Layer<voxblox::EsdfVoxel> &layer = esdf->getEsdfLayer();
  std::optional<sdf_contact_estimation::SdfQueryScope> scope;
  if ( in_scope )
    scope.emplace();

  // Random points over the allocated region and one block beyond it, then
  // points on voxel centres and block faces.
  std::uniform_real_distribution<double> xy( -2.2 * 0.8, 2.2 * 0.8 );
  std::uniform_real_distribution<double> zr( -1.2 * 0.8, 1.2 * 0.8 );
  std::vector<Eigen::Vector3d> points;
  for ( int i = 0; i < 200000; ++i ) points.emplace_back( xy( rng ), xy( rng ), zr( rng ) );
  std::uniform_int_distribution<int> k( -40, 40 );
  for ( int i = 0; i < 20000; ++i ) {
    const Eigen::Vector3d centre( ( k( rng ) + 0.5 ) * 0.05, ( k( rng ) + 0.5 ) * 0.05,
                                  ( k( rng ) / 2 + 0.5 ) * 0.05 );
    const Eigen::Vector3d face( k( rng ) / 10 * 0.8, k( rng ) / 10 * 0.8, k( rng ) / 20 * 0.8 );
    points.push_back( centre );
    points.push_back( face );
    points.emplace_back( face.x(), centre.y(), centre.z() );
  }

  int unknown = 0;
  for ( const Eigen::Vector3d &p : points ) {
    bool ref_unknown = false;
    const Data ref = referenceData( layer, p.x(), p.y(), p.z(), &ref_unknown );
    bool got_unknown = !ref_unknown;
    const Data got =
        interpolator.GetInterpolationVoxelData( p.x(), p.y(), p.z(), 1, &got_unknown );
    expectSameData( got, ref );
    EXPECT_EQ( got_unknown, ref_unknown );
    unknown += ref_unknown;

    const double tx = ( p.x() - ref.x1 ) / ( ref.x2 - ref.x1 );
    const double ty = ( p.y() - ref.y1 ) / ( ref.y2 - ref.y1 );
    const double tz = ( p.z() - ref.z1 ) / ( ref.z2 - ref.z1 );
    const double ref_sdf = interpolator.LinearInterpolation( tx, ty, tz, ref );
    const Eigen::Vector3d ref_gradient = interpolator.LinearInterpolationGradient( tx, ty, tz, ref );
    Eigen::Vector3d gradient;
    EXPECT_EQ( interpolator.GetSDF( p.x(), p.y(), p.z(), 1 ), ref_sdf );
    EXPECT_EQ( interpolator.GetSDFAndGradient( p.x(), p.y(), p.z(), gradient, 1 ), ref_sdf );
    EXPECT_EQ( gradient, ref_gradient );
    if ( HasFailure() )
      FAIL() << "at " << p.transpose();
  }
  // Both cases occur often.
  EXPECT_GT( unknown, static_cast<int>( points.size() ) / 10 );
  EXPECT_LT( unknown, static_cast<int>( points.size() ) * 9 / 10 );
}

INSTANTIATE_TEST_SUITE_P( DenseGridAndScope, InterpolatedEsdfModes,
                          ::testing::Combine( ::testing::Bool(), ::testing::Bool() ) );

TEST_P( InterpolatedEsdfModes, EmptyLayerReadsTruncation )
{
  voxblox::EsdfMap::Config config;
  const InterpolatedVoxbloxESDF interpolator( std::make_shared<voxblox::EsdfMap>( config ),
                                              kTruncation, false, true,
                                              std::get<0>( GetParam() ) );
  bool unknown = false;
  EXPECT_FLOAT_EQ( interpolator.GetSDF( 0.1, -0.3, 0.2, 1, &unknown ), kTruncation );
  EXPECT_TRUE( unknown );
}
