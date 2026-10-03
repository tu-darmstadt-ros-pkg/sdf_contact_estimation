#include <boost/filesystem/path.hpp>
#include <sdf_contact_estimation/sdf/sdf_model.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_msgs/msg/mesh.hpp>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <sdf_contact_estimation/test_scenarios.h>
#include <sdf_contact_estimation/util/utils.h>
#include <sdf_contact_estimation/visualisation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sdf_contact_estimation
{

SdfModel::SdfModel() : sdf_type_( NONE ), publishing_( false ) { }

SdfModel::SdfModel( const rclcpp::Node::SharedPtr node ) : SdfModel()
{
  publishing_ = true;
  world_frame_ = "world";
  node_ = node;

  // Transient local so late-joining subscribers (e.g. RViz) still receive the
  // one-shot visualization messages published at load time (latched in ROS 1).
  // Depth 1 so only the latest message per topic is kept and replayed.
  auto qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).transient_local();
  cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>( "cloud", qos );
  mesh_pub_ = node_->create_publisher<voxblox_msgs::msg::Mesh>( "mesh", qos );
  tsdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>( "tsdf_slice", qos );
  esdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>( "esdf_slice", qos );
}

bool SdfModel::loadFromServer( const rclcpp::Node::SharedPtr node )
{
  // Retrieving parameters from subnodes in ros2 jazzy does not work as expected.
  // Therefore, we get the sub namespace and prepend it to the parameter names.
  auto sub_ns = node->get_sub_namespace();
  std::string prefix = sub_ns.empty() ? "" : sub_ns + ".";
  // ROS 2 parameters instead of nh.param
  double truncation_distance_param =
      node->declare_parameter<double>( prefix + "truncation_distance", 0.4 );
  float truncation_distance = static_cast<float>( truncation_distance_param );

  bool use_esdf = node->declare_parameter<bool>( prefix + "use_esdf", true );

  // Generate environment
  std::string sdf_file_path;
  if ( node->get_parameter( prefix + "sdf_file_path", sdf_file_path ) && !sdf_file_path.empty() ) {
    // Load from SDF
    if ( !loadSdfFromFile( sdf_file_path, truncation_distance ) ) {
      return false;
    }
  } else {
    // Create from cloud. Declare-if-absent so a caller may pre-set these
    // parameters (e.g. an override of the scenario / step height) before
    // loadFromServer() runs.
    std::string scenario =
        node->has_parameter( prefix + "scenario" )
            ? node->get_parameter( prefix + "scenario" ).as_string()
            : node->declare_parameter<std::string>( prefix + "scenario", "flat" );
    double voxel_size_param = node->has_parameter( prefix + "voxel_size" )
                                  ? node->get_parameter( prefix + "voxel_size" ).as_double()
                                  : node->declare_parameter<double>( prefix + "voxel_size", 0.05 );
    float voxel_size = static_cast<float>( voxel_size_param );

    pcl::PointCloud<pcl::PointXYZ> cloud = sdf_contact_estimation::createScenarioFromName( scenario );
    loadCloud( cloud, truncation_distance, voxel_size, use_esdf );
  }

  return true;
}

void SdfModel::loadCloud( const pcl::PointCloud<pcl::PointXYZ> &cloud, float truncation_distance,
                          float voxel_size, bool use_esdf )
{
  if ( publishing_ && cloud_pub_ ) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg( cloud, msg );
    msg.header.frame_id = world_frame_;
    if ( node_ ) {
      msg.header.stamp = node_->now();
    }
    cloud_pub_->publish( msg );
  }

  std::shared_ptr<voxblox::TsdfMap> tsdf = computeTsdf( cloud, truncation_distance, voxel_size );
  loadTsdf( tsdf, truncation_distance, use_esdf );
}

bool SdfModel::loadSdfFromFile( std::string file_path, float truncation_distance, bool publish_mesh )
{
  boost::filesystem::path p( file_path );
  std::string extension = p.extension().string();
  if ( extension == ".tsdf" ) {
    return loadTsdfFromFile( file_path, truncation_distance, true, publish_mesh );
  }
  if ( extension == ".esdf" ) {
    return loadEsdfFromFile( file_path, truncation_distance, publish_mesh );
  }
  RCLCPP_ERROR_STREAM( node_->get_logger(), "Unknown file extension '" << extension << "'." );
  return false;
}

void SdfModel::loadTsdf( const std::shared_ptr<voxblox::TsdfMap> &tsdf, float truncation_distance,
                         bool compute_esdf, bool publish_mesh )
{
  if ( compute_esdf ) {
    std::shared_ptr<voxblox::EsdfMap> esdf = computeEsdf( tsdf );
    loadEsdf( esdf, truncation_distance, false );
  } else {
    setTsdfMap( tsdf, truncation_distance );
  }

  if ( publishing_ ) {
    publishTsdfSlice( tsdf_slice_pub_, tsdf, world_frame_ );
  }

  if ( publish_mesh && publishing_ ) {
    std::shared_ptr<voxblox::MeshLayer> mesh = computeMesh( tsdf );
    publishMesh( mesh_pub_, mesh, world_frame_ );
  }
}

bool SdfModel::loadTsdfFromFile( const std::string &tsdf_file_path, float truncation_distance,
                                 bool compute_esdf, bool publish_mesh )
{
  voxblox::Layer<voxblox::TsdfVoxel>::Ptr layer_ptr;
  if ( voxblox::io::LoadLayer<voxblox::TsdfVoxel>( tsdf_file_path, true, &layer_ptr ) ) {
    std::shared_ptr<voxblox::TsdfMap> tsdf = std::make_shared<voxblox::TsdfMap>( layer_ptr );
    loadTsdf( tsdf, truncation_distance, compute_esdf, publish_mesh );
    return true;
  } else {
    RCLCPP_ERROR_STREAM( node_->get_logger(),
                         "Failed to load TSDF from file '" << tsdf_file_path << "'." );
    return false;
  }
}

void SdfModel::loadEsdf( const std::shared_ptr<voxblox::EsdfMap> &esdf, float truncation_distance,
                         bool publish_mesh )
{
  setEsdfMap( esdf, truncation_distance );

  if ( publishing_ ) {
    publishEsdfSlice( esdf_slice_pub_, esdf, world_frame_ );
  }

  if ( publish_mesh && publishing_ ) {
    std::shared_ptr<voxblox::MeshLayer> mesh = computeMesh( esdf );
    publishMesh( mesh_pub_, mesh, world_frame_ );
  }
}

bool SdfModel::loadEsdfFromFile( const std::string &esdf_file_path, float max_truncation_distance,
                                 bool publish_mesh )
{
  voxblox::Layer<voxblox::EsdfVoxel>::Ptr layer_ptr;
  if ( voxblox::io::LoadLayer<voxblox::EsdfVoxel>( esdf_file_path, true, &layer_ptr ) ) {
    std::shared_ptr<voxblox::EsdfMap> esdf = std::make_shared<voxblox::EsdfMap>( layer_ptr );
    loadEsdf( esdf, max_truncation_distance, publish_mesh );
    return true;
  } else {
    RCLCPP_ERROR_STREAM( node_->get_logger(),
                         "Failed to load ESDF from file '" << esdf_file_path << "'." );
    return false;
  }
}

bool SdfModel::saveTsdfToFile( const std::string &file_path )
{
  std::shared_ptr<InterpolatedTsdf> tsdf = getTsdf();
  if ( tsdf ) {
    voxblox::io::SaveLayer( tsdf->getTSDF()->getTsdfLayer(), file_path );
  } else {
    RCLCPP_WARN_STREAM( node_->get_logger(), "No tsdf loaded yet. Can't save to file." );
    return false;
  }
  return true;
}

bool SdfModel::saveEsdfToFile( const std::string &file_path )
{
  std::shared_ptr<InterpolatedEsdf> esdf = getEsdf();
  if ( esdf ) {
    voxblox::io::SaveLayer( esdf->getESDF()->getEsdfLayer(), file_path );
  } else {
    RCLCPP_WARN_STREAM( node_->get_logger(), "No esdf loaded yet. Can't save to file." );
    return false;
  }
  return true;
}

void SdfModel::setTsdfMap( const std::shared_ptr<voxblox::TsdfMap> &tsdf, float truncation_distance )
{
  tsdf_ = std::make_shared<cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxTSDF>(
      tsdf, truncation_distance, false, true );
  sdf_type_ = TSDF;
}

std::shared_ptr<InterpolatedTsdf> SdfModel::getTsdf() const { return tsdf_; }

void SdfModel::setEsdfMap( const std::shared_ptr<voxblox::EsdfMap> &esdf, float truncation_distance )
{
  esdf_ = std::make_shared<cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxESDF>(
      esdf, truncation_distance, false, true );
  sdf_type_ = ESDF;
}

std::shared_ptr<InterpolatedEsdf> SdfModel::getEsdf() const { return esdf_; }

SdfType SdfModel::getSdfType() const { return sdf_type_; }

std::shared_ptr<voxblox::TsdfMap> SdfModel::computeTsdf( const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                         float truncation_distance, float voxel_size )
{
  voxblox::Pointcloud vox_cloud;
  pclToVoxbloxCloud( cloud, vox_cloud );
  voxblox::Colors vox_colors;
  vox_colors.resize( cloud.size(), voxblox::Color::Gray() );

  voxblox::TsdfMap::Config tsdf_config;
  tsdf_config.tsdf_voxel_size = static_cast<voxblox::FloatingPoint>( voxel_size );

  std::shared_ptr<voxblox::TsdfMap> voxblox_tsdf;
  voxblox_tsdf.reset( new voxblox::TsdfMap( tsdf_config ) );

  voxblox::TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;
  integrator_config.default_truncation_distance = truncation_distance;
  integrator_config.max_ray_length_m = 120.0;

  std::shared_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator;
  tsdf_integrator.reset(
      new voxblox::SimpleTsdfIntegrator( integrator_config, voxblox_tsdf->getTsdfLayerPtr() ) );

  voxblox::Transformation t;
  t.setIdentity();
  t.getPosition().z() = 2.0;

  auto start = std::chrono::high_resolution_clock::now();
  tsdf_integrator->integratePointCloud( t, vox_cloud, vox_colors );
  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;

  RCLCPP_INFO_STREAM( rclcpp::get_logger( "sdf_model" ),
                      "TSDF map update took " << elapsed.count() << " s." );

  return voxblox_tsdf;
}

std::shared_ptr<voxblox::EsdfMap> SdfModel::computeEsdf( const std::shared_ptr<voxblox::TsdfMap> &tsdf )
{
  voxblox::EsdfMap::Config esdf_config;
  esdf_config.esdf_voxel_size = tsdf->getTsdfLayerPtr()->voxel_size();
  esdf_config.esdf_voxels_per_side = tsdf->getTsdfLayerPtr()->voxels_per_side();

  std::shared_ptr<voxblox::EsdfMap> esdf;
  esdf.reset( new voxblox::EsdfMap( esdf_config ) );

  voxblox::EsdfIntegrator::Config esdf_integrator_config;
  esdf_integrator_config.min_distance_m = esdf_config.esdf_voxel_size;
  esdf_integrator_config.max_distance_m = static_cast<voxblox::FloatingPoint>( 100.0 );
  esdf_integrator_config.default_distance_m = static_cast<voxblox::FloatingPoint>( 100.0 );

  std::shared_ptr<voxblox::EsdfIntegrator> esdf_integrator;
  esdf_integrator.reset( new voxblox::EsdfIntegrator(
      esdf_integrator_config, tsdf->getTsdfLayerPtr(), esdf->getEsdfLayerPtr() ) );

  auto start = std::chrono::high_resolution_clock::now();
  esdf_integrator->updateFromTsdfLayerBatch();
  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;

  RCLCPP_INFO_STREAM( rclcpp::get_logger( "sdf_model" ),
                      "ESDF map update took " << elapsed.count() << " s." );

  return esdf;
}

std::shared_ptr<voxblox::MeshLayer>
SdfModel::computeMesh( const std::shared_ptr<voxblox::TsdfMap> &tsdf )
{
  voxblox::MeshIntegratorConfig mesh_config;
  mesh_config.min_weight = 0.1f;
  auto mesh_layer = std::make_shared<voxblox::MeshLayer>( tsdf->block_size() );
  auto mesh_integrator = std::make_shared<voxblox::MeshIntegrator<voxblox::TsdfVoxel>>(
      mesh_config, tsdf->getTsdfLayerPtr(), mesh_layer.get() );

  bool only_mesh_updated_blocks = false;
  bool clear_updated_flag = true;
  mesh_integrator->generateMesh( only_mesh_updated_blocks, clear_updated_flag );
  return mesh_layer;
}

std::shared_ptr<voxblox::MeshLayer>
SdfModel::computeMesh( const std::shared_ptr<voxblox::EsdfMap> &esdf )
{
  voxblox::MeshIntegratorConfig mesh_config;
  mesh_config.min_weight = 0.1f;
  auto mesh_layer = std::make_shared<voxblox::MeshLayer>( esdf->block_size() );
  auto mesh_integrator = std::make_shared<voxblox::MeshIntegrator<voxblox::EsdfVoxel>>(
      mesh_config, esdf->getEsdfLayerPtr(), mesh_layer.get() );

  bool only_mesh_updated_blocks = false;
  bool clear_updated_flag = true;
  mesh_integrator->generateMesh( only_mesh_updated_blocks, clear_updated_flag );
  return mesh_layer;
}

bool SdfModel::isLoaded() const { return getSdfType() != SdfType::NONE; }

bool SdfModel::isEmpty() const
{
  switch ( sdf_type_ ) {
  case NONE:
    return true;
  case TSDF:
    return tsdf_->getTSDF()->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() == 0;
  case ESDF:
    return esdf_->getESDF()->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() == 0;
  }

  return true;
}

void SdfModel::publishSdfMesh() const
{
  std::shared_ptr<voxblox::MeshLayer> mesh;
  if ( sdf_type_ == TSDF ) {
    mesh = computeMesh( tsdf_->getTSDF() );
  } else if ( sdf_type_ == ESDF ) {
    mesh = computeMesh( esdf_->getESDF() );
  }

  if ( mesh ) {
    publishMesh( mesh_pub_, mesh, world_frame_ );
  }
}

void SdfModel::setWorldFrame( const std::string &world_frame ) { world_frame_ = world_frame; }

std::string SdfModel::getWorldFrame() const { return world_frame_; }

} // namespace sdf_contact_estimation
