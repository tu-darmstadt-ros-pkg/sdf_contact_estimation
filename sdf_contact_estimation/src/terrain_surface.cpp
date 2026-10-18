#include <sdf_contact_estimation/sdf/terrain_surface.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <sdf_contact_estimation/sdf/sdf_query_scope.h>

namespace sdf_contact_estimation
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
// Shrinks the bracket of the crossing, at most one voxel, to 1/256 of it.
constexpr int kBisections = 8;

double voxelSize( const SdfModel &sdf )
{
  if ( sdf.getSdfType() == ESDF && sdf.getEsdf() )
    return sdf.getEsdf()->getESDF()->getEsdfLayer().voxel_size();
  if ( sdf.getSdfType() == TSDF && sdf.getTsdf() )
    return sdf.getTsdf()->getTSDF()->getTsdfLayer().voxel_size();
  throw std::invalid_argument( "surfaceHeight: the SDF is not loaded" );
}

double yawOf( const Eigen::Isometry3d &pose )
{
  const Eigen::Vector3d x = pose.linear().col( 0 );
  return std::atan2( x.y(), x.x() );
}

// Rows of surfaceHeight() samples along the path (see surfaceBand), z NaN or
// infinite without a surface. Each infinite sample also goes to no_surface at
// the end of its search.
std::vector<std::vector<Eigen::Vector3d>> sampleRows( const SdfModel &sdf,
                                                      const std::vector<Eigen::Isometry3d> &poses,
                                                      const std::vector<double> &lateral,
                                                      double spacing, double range,
                                                      std::vector<Eigen::Vector3d> &no_surface )
{
  std::vector<std::vector<Eigen::Vector3d>> rows;
  const auto row = [&]( const Eigen::Vector3d &p, double yaw ) {
    const Eigen::Vector3d left( -std::sin( yaw ), std::cos( yaw ), 0.0 );
    std::vector<Eigen::Vector3d> out;
    for ( double offset : lateral ) {
      Eigen::Vector3d q = p + offset * left;
      q.z() = surfaceHeight( sdf, q.x(), q.y(), p.z(), range );
      if ( std::isinf( q.z() ) )
        no_surface.emplace_back( q.x(), q.y(), p.z() + std::copysign( range, q.z() ) );
      out.push_back( q );
    }
    rows.push_back( std::move( out ) );
  };
  // Distance along the path still to cover before the next row.
  double to_next = 0.0;
  for ( size_t i = 0; i + 1 < poses.size(); ++i ) {
    const Eigen::Vector3d a = poses[i].translation();
    const Eigen::Vector3d b = poses[i + 1].translation();
    const double length = ( b - a ).norm();
    const double yaw_a = yawOf( poses[i] );
    const double turn = std::remainder( yawOf( poses[i + 1] ) - yaw_a, 2.0 * M_PI );
    double s = to_next;
    for ( ; s < length; s += spacing ) {
      const double t = s / length;
      row( a + t * ( b - a ), yaw_a + t * turn );
    }
    to_next = s - length;
  }
  row( poses.back().translation(), yawOf( poses.back() ) );
  return rows;
}
} // namespace

double surfaceHeight( const SdfModel &sdf, double x, double y, double z, double range )
{
  const double voxel = voxelSize( sdf );
  if ( !( range > 0.0 ) )
    throw std::invalid_argument( "surfaceHeight: range must be positive" );
  bool touched_unknown = false;
  const auto isFree = [&]( double zq ) {
    bool unknown = false;
    const bool free = sdf.getSdf<double>( x, y, zq, &unknown ) > 0.0;
    touched_unknown = touched_unknown || unknown;
    return free;
  };
  const bool start_free = isFree( z );
  const double dir = start_free ? -1.0 : 1.0;
  const double end = z + dir * range;
  // The interpolation is monotone in z between voxel centre heights, so
  // sampling at these heights finds every crossing, also through thin solid.
  double k = start_free ? std::ceil( z / voxel - 0.5 ) - 1.0 : std::floor( z / voxel - 0.5 ) + 1.0;
  // Bracket of the crossing: the last sample on the start side and the first beyond.
  double inside = z;
  double outside = kNaN;
  for ( ; std::isnan( outside ) && inside != end; k += dir ) {
    const double centre = ( k + 0.5 ) * voxel;
    const double zq = std::abs( centre - z ) < range ? centre : end;
    ( isFree( zq ) == start_free ? inside : outside ) = zq;
  }
  if ( std::isnan( outside ) )
    return touched_unknown ? kNaN : dir * kInf;
  for ( int i = 0; i < kBisections; ++i ) {
    const double mid = 0.5 * ( inside + outside );
    ( isFree( mid ) == start_free ? inside : outside ) = mid;
  }
  const double height = 0.5 * ( inside + outside );
  bool unknown = false;
  sdf.getSdf<double>( x, y, height, &unknown );
  return unknown ? kNaN : height;
}

SurfaceBand surfaceBand( const SdfModel &sdf, const std::vector<Eigen::Isometry3d> &poses,
                         const std::vector<std::vector<double>> &tracks, double spacing, double range )
{
  if ( spacing <= 0.0 )
    throw std::invalid_argument( "surfaceBand: spacing must be positive" );
  SurfaceBand band;
  if ( poses.empty() )
    return band;
  const SdfQueryScope scope;
  for ( const std::vector<double> &lateral : tracks ) {
    const std::vector<std::vector<Eigen::Vector3d>> rows =
        sampleRows( sdf, poses, lateral, spacing, range, band.no_surface );
    for ( size_t i = 1; i < rows.size(); ++i ) {
      const std::vector<Eigen::Vector3d> &a = rows[i - 1];
      const std::vector<Eigen::Vector3d> &b = rows[i];
      for ( size_t j = 1; j < lateral.size(); ++j ) {
        if ( !( a[j - 1].allFinite() && a[j].allFinite() && b[j - 1].allFinite() && b[j].allFinite() ) )
          continue;
        band.triangles.insert( band.triangles.end(),
                               { a[j - 1], b[j - 1], b[j], a[j - 1], b[j], a[j] } );
      }
    }
  }
  return band;
}

std::vector<std::vector<double>> trackOffsets( const std::vector<Eigen::Vector3d> &track_points,
                                               int count )
{
  std::vector<std::vector<double>> tracks;
  for ( double side : { 1.0, -1.0 } ) {
    double inner = std::numeric_limits<double>::infinity();
    double outer = -std::numeric_limits<double>::infinity();
    for ( const Eigen::Vector3d &p : track_points ) {
      if ( side * p.y() > 0.0 ) {
        inner = std::min( inner, side * p.y() );
        outer = std::max( outer, side * p.y() );
      }
    }
    std::vector<double> offsets;
    for ( int k = 0; std::isfinite( inner ) && k < count; ++k )
      offsets.push_back( side *
                         ( count > 1 ? inner + ( outer - inner ) * k / ( count - 1 ) : inner ) );
    tracks.push_back( std::move( offsets ) );
  }
  return tracks;
}

} // namespace sdf_contact_estimation
