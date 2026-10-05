#ifndef SDF_CONTACT_ESTIMATION_TEST_HELPERS_H
#define SDF_CONTACT_ESTIMATION_TEST_HELPERS_H

// Test fixtures: a minimal two-track robot from test/data and ESDF maps built
// voxel by voxel.

#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <voxblox/core/esdf_map.h>

#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/sdf_contact_estimation.h>

namespace sdf_contact_estimation
{
namespace test
{

inline std::string readFile( const std::string &path )
{
  std::ifstream in( path );
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

inline ShapeModelPtr makeTestRobot()
{
  ShapeModelConfig config;
  config.urdf = readFile( std::string( TEST_DATA_DIR ) + "/test_robot.urdf" );
  config.collision_links_config_file = std::string( TEST_DATA_DIR ) + "/test_robot_collision.yaml";
  config.default_resolution = 0.05;
  return std::make_shared<ShapeModel>( config );
}

inline SdfContactEstimationSettings makeSettings()
{
  // Mirrors the planner's solver settings (common_solver.yaml).
  SdfContactEstimationSettings settings( 5, 0.05, 1.0472, false, 0.0 );
  settings.iteration_contact_threshold = 0.025;
  settings.chassis_contact_threshold = 0.025;
  return settings;
}

/// Voxel classification used by makeFloorEsdf().
enum class VoxelState { Observed, Unobserved, NoBlock };

/// ESDF of a horizontal floor at z = floor_z, observed in x, y in [-extent, extent),
/// z in [-0.8, 0.8). `state(x, y)` marks columns (voxel centers) as unobserved
/// (voxels allocated, observed = false) or as missing (whole block left out).
/// A block is left out only if all its columns are NoBlock.
inline std::shared_ptr<voxblox::EsdfMap>
makeFloorEsdf( double floor_z = 0.0, double extent = 1.6,
               const std::function<VoxelState( double, double )> &state = nullptr )
{
  voxblox::EsdfMap::Config config;
  config.esdf_voxel_size = 0.05;
  config.esdf_voxels_per_side = 16;
  auto esdf = std::make_shared<voxblox::EsdfMap>( config );
  voxblox::Layer<voxblox::EsdfVoxel> *layer = esdf->getEsdfLayerPtr();
  const double block_size = layer->block_size();
  const int n_xy = static_cast<int>( std::round( extent / block_size ) );
  const size_t vps = layer->voxels_per_side();
  for ( int bx = -n_xy; bx < n_xy; ++bx ) {
    for ( int by = -n_xy; by < n_xy; ++by ) {
      for ( int bz = -1; bz < 1; ++bz ) {
        const voxblox::BlockIndex block_index( bx, by, bz );
        bool all_missing = static_cast<bool>( state );
        for ( size_t i = 0; all_missing && i < vps; ++i ) {
          for ( size_t j = 0; all_missing && j < vps; ++j ) {
            const double x = ( bx * static_cast<double>( vps ) + i + 0.5 ) * layer->voxel_size();
            const double y = ( by * static_cast<double>( vps ) + j + 0.5 ) * layer->voxel_size();
            all_missing = state( x, y ) == VoxelState::NoBlock;
          }
        }
        if ( all_missing )
          continue;
        auto block = layer->allocateBlockPtrByIndex( block_index );
        for ( size_t linear = 0; linear < block->num_voxels(); ++linear ) {
          const voxblox::Point c = block->computeCoordinatesFromLinearIndex( linear );
          voxblox::EsdfVoxel &voxel = block->getVoxelByLinearIndex( linear );
          voxel.distance = static_cast<float>( c.z() - floor_z );
          voxel.observed = !state || state( c.x(), c.y() ) == VoxelState::Observed;
        }
      }
    }
  }
  return esdf;
}

inline SdfModelPtr makeSdfModel( const std::shared_ptr<voxblox::EsdfMap> &esdf )
{
  auto model = std::make_shared<SdfModel>();
  model->loadEsdf( esdf, 0.4f, false );
  return model;
}

} // namespace test
} // namespace sdf_contact_estimation

#endif // SDF_CONTACT_ESTIMATION_TEST_HELPERS_H
