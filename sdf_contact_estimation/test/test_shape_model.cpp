#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "test_helpers.h"

using namespace sdf_contact_estimation;
using sdf_contact_estimation::test::readFile;

namespace
{

/// Height range of the sampling points of the test robot's body box (centre z
/// 0.15, height 0.1), with or without invert_z on it.
std::pair<double, double> bodyBoxHeights( bool invert_z )
{
  const std::string path = testing::TempDir() + "/body_box.yaml";
  std::ofstream( path ) << "collision_links:\n- link: base_link\n  type: body\n  resolution: 0.05\n"
                        << "  include_indices: [2]\n"
                        << ( invert_z ? "  invert_z: [2]\n" : "" );
  ShapeModelConfig config;
  config.urdf = readFile( std::string( TEST_DATA_DIR ) + "/test_robot.urdf" );
  config.collision_links_config_file = path;
  const ShapeModel model( config );
  double lo = 1e9, hi = -1e9;
  for ( const ShapePtr &shape : model.getShape() )
    for ( const Eigen::Vector3d &p : shape->getSamplingPoints() ) {
      lo = std::min( lo, p.z() );
      hi = std::max( hi, p.z() );
    }
  std::remove( path.c_str() );
  return { lo, hi };
}

} // namespace

TEST( ShapeModel, BoxIsSampledOnItsBottomFace )
{
  const auto [lo, hi] = bodyBoxHeights( false );
  EXPECT_NEAR( lo, 0.10, 1e-9 );
  EXPECT_NEAR( hi, 0.10, 1e-9 );
}

TEST( ShapeModel, InvertZSamplesTheBoxTopFace )
{
  const auto [lo, hi] = bodyBoxHeights( true );
  EXPECT_NEAR( lo, 0.20, 1e-9 );
  EXPECT_NEAR( hi, 0.20, 1e-9 );
}
