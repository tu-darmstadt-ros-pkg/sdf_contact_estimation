/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_VOXBLOX_ESDF_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_VOXBLOX_ESDF_H_

#include <cmath>
#include <cstdint>
#include <vector>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <voxblox/core/common.h>
#include <voxblox/core/esdf_map.h>

namespace cartographer
{
namespace mapping_3d
{
namespace scan_matching
{

// Interpolates between HybridGrid probability voxels. We use the tricubic
// interpolation which interpolates the values and has vanishing derivative at
// these points.
//
// This class is templated to work with the autodiff that Ceres provides.
// For this reason, it is also important that the interpolation scheme be
// continuously differentiable.

// Reads the layer from a dense copy made at construction, so later changes to
// the layer are not seen.
class InterpolatedVoxbloxESDF
{
public:
  explicit InterpolatedVoxbloxESDF( const std::shared_ptr<voxblox::EsdfMap> tsdf,
                                    float max_truncation_distance,
                                    bool use_cubic_interpolation = true,
                                    bool use_boundary_extrapolation = true )
      : esdf_( tsdf ), max_truncation_distance_( max_truncation_distance ),
        use_cubic_interpolation_( use_cubic_interpolation ),
        use_boundary_extrapolation_( use_boundary_extrapolation ),
        voxel_size_inv_( tsdf->getEsdfLayer().voxel_size_inv() )
  {
    buildGrid();
  }

  InterpolatedVoxbloxESDF( const InterpolatedVoxbloxESDF & ) = delete;
  InterpolatedVoxbloxESDF &operator=( const InterpolatedVoxbloxESDF & ) = delete;

  // Returns the interpolated probability at (x, y, z) of the HybridGrid
  // used to perform the interpolation.
  //
  // This is a piecewise, continuously differentiable function. We use the
  // scalar part of Jet parameters to select our interval below. It is the
  // tensor product volume of piecewise cubic polynomials that interpolate
  // the values, and have vanishing derivative at the interval boundaries.

  // Coordinates and values of the eight neighboring voxels
  struct InterpolationData {
    double x1, y1, z1, x2, y2, z2;
    double q111, q112, q121, q122, q211, q212, q221, q222;
  };

  // If given, *touched_unknown is set to whether any of the eight corners is
  // unobserved or lies in a missing block.
  template<typename T>
  InterpolationData GetInterpolationVoxelData( const T &x, const T &y, const T &z,
                                               int coarsening_factor,
                                               bool *touched_unknown = nullptr ) const
  {
    InterpolationData values{};

    // Get voxel coordinates
    ComputeInterpolationDataPoints( x, y, z, &values.x1, &values.y1, &values.z1, &values.x2,
                                    &values.y2, &values.z2, coarsening_factor );

    // Corner values in the order q111, q112, ..., q222.
    const int ix[2] = { gridIndex( values.x1 ), gridIndex( values.x2 ) };
    const int iy[2] = { gridIndex( values.y1 ), gridIndex( values.y2 ) };
    const int iz[2] = { gridIndex( values.z1 ), gridIndex( values.z2 ) };
    double *q[8] = { &values.q111, &values.q112, &values.q121, &values.q122,
                     &values.q211, &values.q212, &values.q221, &values.q222 };
    size_t num_invalid_voxel = 0;
    double summed_valid_sdf = 0.0;
    for ( int c = 0; c < 8; ++c ) {
      *q[c] = gridSDF( ix[c >> 2], iy[( c >> 1 ) & 1], iz[c & 1] );
      if ( std::isnan( *q[c] ) ) {
        num_invalid_voxel++;
      } else {
        summed_valid_sdf += *q[c];
      }
    }

    if ( touched_unknown != nullptr ) {
      *touched_unknown = num_invalid_voxel > 0;
    }
    if ( num_invalid_voxel > 0 ) {
      double signed_max_tsdf =
          summed_valid_sdf < 0 ? -max_truncation_distance_ : max_truncation_distance_;
      if ( use_boundary_extrapolation_ ) {
        if ( std::isnan( values.q111 ) ) {
          values.q111 = signed_max_tsdf;
        }
        if ( std::isnan( values.q112 ) ) {
          values.q112 = signed_max_tsdf;
        }
        if ( std::isnan( values.q121 ) ) {
          values.q121 = signed_max_tsdf;
        }
        if ( std::isnan( values.q122 ) ) {
          values.q122 = signed_max_tsdf;
        }
        if ( std::isnan( values.q211 ) ) {
          values.q211 = signed_max_tsdf;
        }
        if ( std::isnan( values.q212 ) ) {
          values.q212 = signed_max_tsdf;
        }
        if ( std::isnan( values.q221 ) ) {
          values.q221 = signed_max_tsdf;
        }
        if ( std::isnan( values.q222 ) ) {
          values.q222 = signed_max_tsdf;
        }
      } else {
        values.q111 = max_truncation_distance_;
        values.q112 = max_truncation_distance_;
        values.q121 = max_truncation_distance_;
        values.q122 = max_truncation_distance_;
        values.q211 = max_truncation_distance_;
        values.q212 = max_truncation_distance_;
        values.q221 = max_truncation_distance_;
        values.q222 = max_truncation_distance_;
      }
    }
    return values;
  }

  template<typename T>
  T CubicInterpolation( T tx, T ty, T tz, const InterpolationData &values ) const
  {
    // Compute pow(..., 2) and pow(..., 3). Using pow() here is very expensive.
    const T normalized_xx = tx * tx;
    const T normalized_xxx = tx * normalized_xx;
    const T normalized_yy = ty * ty;
    const T normalized_yyy = ty * normalized_yy;
    const T normalized_zz = tz * tz;
    const T normalized_zzz = tz * normalized_zz;

    // We first interpolate in z, then y, then x. All 7 times this uses the same
    // scheme: A * (2t^3 - 3t^2 + 1) + B * (-2t^3 + 3t^2).
    // The first polynomial is 1 at t=0, 0 at t=1, the second polynomial is 0
    // at t=0, 1 at t=1. Both polynomials have derivative zero at t=0 and t=1.
    const T q11 = ( values.q111 - values.q112 ) * normalized_zzz * 2. +
                  ( values.q112 - values.q111 ) * normalized_zz * 3. + values.q111;
    const T q12 = ( values.q121 - values.q122 ) * normalized_zzz * 2. +
                  ( values.q122 - values.q121 ) * normalized_zz * 3. + values.q121;
    const T q21 = ( values.q211 - values.q212 ) * normalized_zzz * 2. +
                  ( values.q212 - values.q211 ) * normalized_zz * 3. + values.q211;
    const T q22 = ( values.q221 - values.q222 ) * normalized_zzz * 2. +
                  ( values.q222 - values.q221 ) * normalized_zz * 3. + values.q221;
    const T q1 = ( q11 - q12 ) * normalized_yyy * 2. + ( q12 - q11 ) * normalized_yy * 3. + q11;
    const T q2 = ( q21 - q22 ) * normalized_yyy * 2. + ( q22 - q21 ) * normalized_yy * 3. + q21;
    return ( q1 - q2 ) * normalized_xxx * 2. + ( q2 - q1 ) * normalized_xx * 3. + q1;
  }

  template<typename T>
  T LinearInterpolation( T tx, T ty, T tz, const InterpolationData &values ) const
  {
    const T c00 = values.q111 * ( T( 1.0 ) - tx ) + values.q211 * tx;
    const T c01 = values.q112 * ( T( 1.0 ) - tx ) + values.q212 * tx;
    const T c10 = values.q121 * ( T( 1.0 ) - tx ) + values.q221 * tx;
    const T c11 = values.q122 * ( T( 1.0 ) - tx ) + values.q222 * tx;

    const T c0 = c00 * ( T( 1.0 ) - ty ) + c10 * ty;
    const T c1 = c01 * ( T( 1.0 ) - ty ) + c11 * ty;

    const T c = c0 * ( T( 1.0 ) - tz ) + c1 * tz;
    return c;
  }

  template<typename T>
  Eigen::Matrix<T, 3, 1> LinearInterpolationGradient( T tx, T ty, T tz,
                                                      const InterpolationData &values ) const
  {
    // Partial derivatives with respect to tx, ty, tz
    const T dfdtx = ( T( 1.0 ) - ty ) * ( T( 1.0 ) - tz ) * ( values.q211 - values.q111 ) +
                    ( T( 1.0 ) - ty ) * tz * ( values.q212 - values.q112 ) +
                    ty * ( T( 1.0 ) - tz ) * ( values.q221 - values.q121 ) +
                    ty * tz * ( values.q222 - values.q122 );

    const T dfdty = ( T( 1.0 ) - tx ) * ( T( 1.0 ) - tz ) * ( values.q121 - values.q111 ) +
                    ( T( 1.0 ) - tx ) * tz * ( values.q122 - values.q112 ) +
                    tx * ( T( 1.0 ) - tz ) * ( values.q221 - values.q211 ) +
                    tx * tz * ( values.q222 - values.q212 );

    const T dfdtz = ( T( 1.0 ) - tx ) * ( T( 1.0 ) - ty ) * ( values.q112 - values.q111 ) +
                    ( T( 1.0 ) - tx ) * ty * ( values.q122 - values.q121 ) +
                    tx * ( T( 1.0 ) - ty ) * ( values.q212 - values.q211 ) +
                    tx * ty * ( values.q222 - values.q221 );

    // Convert partial derivatives with respect to tx, ty, tz to x, y, z
    const T dfdx = dfdtx / ( values.x2 - values.x1 );
    const T dfdy = dfdty / ( values.y2 - values.y1 );
    const T dfdz = dfdtz / ( values.z2 - values.z1 );
    return { dfdx, dfdy, dfdz };
  }

  // touched_unknown: optional, see GetInterpolationVoxelData().
  template<typename T>
  T GetSDF( const T &x, const T &y, const T &z, int coarsening_factor,
            bool *touched_unknown = nullptr ) const
  {
    // const auto& chunk_manager = tsdf_->GetChunkManager(); //todo(kdaun) reenable
    // chisel::Vec3 origin = chunk_manager.GetOrigin();
    T x_local = x; // - T(origin.x());
    T y_local = y; // - T(origin.y());
    T z_local = z; // - T(origin.z());

    InterpolationData values =
        GetInterpolationVoxelData( x_local, y_local, z_local, coarsening_factor, touched_unknown );

    const T tx = ( x - values.x1 ) / ( values.x2 - values.x1 );
    const T ty = ( y - values.y1 ) / ( values.y2 - values.y1 );
    const T tz = ( z - values.z1 ) / ( values.z2 - values.z1 );

    if ( use_cubic_interpolation_ ) {
      return CubicInterpolation( tx, ty, tz, values );
    } else { // trilinear interpolation https://en.wikipedia.org/wiki/Trilinear_interpolation
      return LinearInterpolation( tx, ty, tz, values );
    }
  }

  template<typename T>
  T GetSDFAndGradient( const T &x, const T &y, const T &z, Eigen::Matrix<T, 3, 1> &gradient,
                       int coarsening_factor, bool *touched_unknown = nullptr ) const
  {
    // const auto& chunk_manager = tsdf_->GetChunkManager(); //todo(kdaun) reenable
    // chisel::Vec3 origin = chunk_manager.GetOrigin();
    T x_local = x; // - T(origin.x());
    T y_local = y; // - T(origin.y());
    T z_local = z; // - T(origin.z());

    InterpolationData values =
        GetInterpolationVoxelData( x_local, y_local, z_local, coarsening_factor, touched_unknown );

    const T tx = ( x - values.x1 ) / ( values.x2 - values.x1 );
    const T ty = ( y - values.y1 ) / ( values.y2 - values.y1 );
    const T tz = ( z - values.z1 ) / ( values.z2 - values.z1 );

    T distance;
    if ( use_cubic_interpolation_ ) {
      distance = CubicInterpolation( tx, ty, tz, values );
      gradient = Eigen::Matrix<T, 3, 1>::Zero();
      RCLCPP_ERROR_STREAM( rclcpp::get_logger( "GetSDFAndGradient" ),
                           "Gradient computation not implemented for cubic interpolation" );
    } else { // trilinear interpolation https://en.wikipedia.org/wiki/Trilinear_interpolation
      distance = LinearInterpolation( tx, ty, tz, values );
      gradient = LinearInterpolationGradient( tx, ty, tz, values );
    }
    return distance;
  }

  const std::shared_ptr<voxblox::EsdfMap> getESDF() const { return esdf_; }

  float getTruncationDistance() const { return max_truncation_distance_; }

private:
  template<typename T>
  void ComputeInterpolationDataPoints( const T &x, const T &y, const T &z, double *x1, double *y1,
                                       double *z1, double *x2, double *y2, double *z2,
                                       int coarsening_factor ) const
  {
    const Eigen::Vector3f lower = CenterOfLowerVoxel( x, y, z, coarsening_factor );
    // const auto& chunk_manager = tsdf_->GetChunkManager();
    const float resolution =
        esdf_->getEsdfLayer()
            .voxel_size(); // todo(kdaun) handle different resolutions for different directions
    // LOG(INFO)<<"resolution "<<resolution;
    *x1 = lower.x();
    *y1 = lower.y();
    *z1 = lower.z();
    *x2 = lower.x() + resolution;
    *y2 = lower.y() + resolution;
    *z2 = lower.z() + resolution;
  }

  // Center of the next lower voxel, i.e., not necessarily the voxel containing
  // (x, y, z). For each dimension, the largest voxel index so that the
  // corresponding center is at most the given coordinate.
  Eigen::Vector3f CenterOfLowerVoxel( const double x, const double y, const double z,
                                      int coarsening_factor ) const
  {
    const float coarsed_resolution = esdf_->getEsdfLayer().voxel_size() * coarsening_factor;
    // LOG(INFO)<<"resolution "<< tsdf_->getTsdfLayer().voxel_size();
    const float round = 1 / coarsed_resolution;
    const float offset = 0.5 * coarsed_resolution;
    float x_0 = static_cast<float>( ( std::floor( x * round + 0.5 ) / round ) - offset );
    float y_0 = static_cast<float>( ( std::floor( y * round + 0.5 ) / round ) - offset );
    float z_0 = static_cast<float>( ( std::floor( z * round + 0.5 ) / round ) - offset );
    Eigen::Vector3f center( x_0, y_0, z_0 );
    return center;
  }

  // Uses the scalar part of a Ceres Jet.
  template<typename T>
  Eigen::Vector3f CenterOfLowerVoxel( const T &jet_x, const T &jet_y, const T &jet_z,
                                      int coarsening_factor ) const
  {
    return CenterOfLowerVoxel( jet_x.a, jet_y.a, jet_z.a, coarsening_factor );
  }

  // Global voxel index along one axis of the voxel containing coordinate x [m],
  // computed as voxblox does.
  int gridIndex( double x ) const
  {
    return static_cast<int>(
        std::floor( static_cast<float>( x ) * voxel_size_inv_ + voxblox::kEpsilon ) );
  }

  // Distance of the voxel at global index (x, y, z), NaN if it is unobserved
  // or outside the grid.
  double gridSDF( int x, int y, int z ) const
  {
    const unsigned gx = static_cast<unsigned>( x - grid_min_.x() );
    const unsigned gy = static_cast<unsigned>( y - grid_min_.y() );
    const unsigned gz = static_cast<unsigned>( z - grid_min_.z() );
    if ( gx >= static_cast<unsigned>( grid_size_.x() ) ||
         gy >= static_cast<unsigned>( grid_size_.y() ) ||
         gz >= static_cast<unsigned>( grid_size_.z() ) ) {
      return NAN;
    }
    return grid_[( static_cast<size_t>( gz ) * grid_size_.y() + gy ) * grid_size_.x() + gx];
  }

  // Copies the layer into grid_, a dense array over the bounding box of its
  // allocated blocks (x fastest), with NaN for unobserved voxels.
  void buildGrid()
  {
    const voxblox::Layer<voxblox::EsdfVoxel> &layer = esdf_->getEsdfLayer();
    voxblox::BlockIndexList blocks;
    layer.getAllAllocatedBlocks( &blocks );
    if ( blocks.empty() ) {
      return;
    }
    const int vps = static_cast<int>( layer.voxels_per_side() );
    voxblox::BlockIndex lo = blocks.front(), hi = blocks.front();
    for ( const voxblox::BlockIndex &b : blocks ) {
      lo = lo.cwiseMin( b );
      hi = hi.cwiseMax( b );
    }
    grid_min_ = lo * vps;
    grid_size_ = ( hi - lo + voxblox::BlockIndex::Ones() ) * vps;
    grid_.assign( static_cast<size_t>( grid_size_.x() ) * grid_size_.y() * grid_size_.z(), NAN );
    for ( const voxblox::BlockIndex &b : blocks ) {
      const voxblox::Block<voxblox::EsdfVoxel> &block = layer.getBlockByIndex( b );
      for ( size_t linear = 0; linear < block.num_voxels(); ++linear ) {
        const voxblox::EsdfVoxel &voxel = block.getVoxelByLinearIndex( linear );
        if ( !voxel.observed ) {
          continue;
        }
        const voxblox::VoxelIndex g =
            ( b - lo ) * vps + block.computeVoxelIndexFromLinearIndex( linear );
        grid_[( static_cast<size_t>( g.z() ) * grid_size_.y() + g.y() ) * grid_size_.x() + g.x()] =
            voxel.distance;
      }
    }
  }

  const std::shared_ptr<voxblox::EsdfMap> esdf_;
  float max_truncation_distance_;
  const bool use_cubic_interpolation_;
  const bool use_boundary_extrapolation_;
  const float voxel_size_inv_;
  std::vector<float> grid_;
  voxblox::VoxelIndex grid_min_ = voxblox::VoxelIndex::Zero(); // global index of grid_[0]
  voxblox::VoxelIndex grid_size_ = voxblox::VoxelIndex::Zero(); // [voxels] per axis
};

} // namespace scan_matching
} // namespace mapping_3d
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_VOXBLOX_ESDF_H_
