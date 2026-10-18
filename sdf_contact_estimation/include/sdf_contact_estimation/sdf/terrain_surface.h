#ifndef SDF_CONTACT_ESTIMATION_TERRAIN_SURFACE_H
#define SDF_CONTACT_ESTIMATION_TERRAIN_SURFACE_H

// The terrain surface as an SDF consumer sees it, sampled for display.

#include <vector>

#include <Eigen/Geometry>

#include <sdf_contact_estimation/sdf/sdf_model.h>

namespace sdf_contact_estimation
{

/// Height of the SDF zero level in column (x, y) nearest to z, searched
/// downwards from z when z lies in free space and upwards when it lies in
/// solid, at most range [m] away. -infinity when the column is known free down
/// to z - range, +infinity when it is known solid up to z + range. NaN when the
/// surface interpolation touches unknown voxels or the search ends in unknown
/// space. Throws std::invalid_argument when sdf is not loaded or range is not
/// positive.
double surfaceHeight( const SdfModel &sdf, double x, double y, double z, double range );

/// The terrain under tracks along a path, for display.
struct SurfaceBand {
  /// Three points per triangle, two per cell of neighbouring samples with all
  /// four corners on the surface.
  std::vector<Eigen::Vector3d> triangles;
  /// Samples without a surface within range (surfaceHeight infinite), at the
  /// end of their search.
  std::vector<Eigen::Vector3d> no_surface;
};

/// Samples surfaceHeight() along a path, starting each search at the path
/// height, one band per entry of tracks: a row every spacing [m] of the
/// polyline through the pose positions (the last pose always gets one), one
/// sample per lateral offset [m] of the track along the left axis of the pose
/// heading (yaw interpolated between poses). Unknown samples leave gaps.
/// Throws std::invalid_argument when spacing is not positive.
SurfaceBand surfaceBand( const SdfModel &sdf, const std::vector<Eigen::Isometry3d> &poses,
                         const std::vector<std::vector<double>> &tracks, double spacing,
                         double range );

/// count lateral offsets spanning each track: the left (y > 0) and the right
/// (y < 0) points of track_points (base frame) from their innermost to their
/// outermost y. A side without points gets none.
std::vector<std::vector<double>> trackOffsets( const std::vector<Eigen::Vector3d> &track_points,
                                               int count );

} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_TERRAIN_SURFACE_H
