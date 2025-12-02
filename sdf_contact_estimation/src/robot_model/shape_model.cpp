#include <sdf_contact_estimation/robot_model/shape_model.h>

#include <sdf_contact_estimation/robot_model/basic_shapes/cylinder_shape.h>
#include <sdf_contact_estimation/robot_model/basic_shapes/rectangle_shape.h>
#include <sdf_contact_estimation/util/utils.h>

#include <geometric_shapes/bodies.h>
#include <moveit/robot_state/conversions.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

namespace sdf_contact_estimation
{

ShapeModel::ShapeModel( const rclcpp::Node::SharedPtr &node )
    : RobotModel( std::unordered_map<std::string, double>() ), total_sampling_point_count_( 0 ),
      robot_mass_( 0.0 ), node_( node )
{
  loadParameters( node );
  loadRobotModel( node );
  generateShape();
}

const RobotShape &ShapeModel::getShape() const { return shape_; }

size_t ShapeModel::getTotalSamplingPointCount() const { return total_sampling_point_count_; }

void ShapeModel::getRobotStateVisualization( visualization_msgs::msg::MarkerArray &marker_array,
                                             const Eigen::Isometry3d &pose,
                                             const std::string &frame_id ) const
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.8;
  color.b = 1.0;
  visualization_msgs::msg::MarkerArray marker_array_temp;
  robot_state_->getRobotMarkers( marker_array_temp, robot_model_->getLinkModelNames(), color,
                                 "robot_state", rclcpp::Duration::from_seconds( 0.0 ) );
  // Transform from base to world frame
  for ( auto &marker : marker_array_temp.markers ) {
    Eigen::Isometry3d marker_pose_base;
    tf2::fromMsg( marker.pose, marker_pose_base );
    Eigen::Isometry3d marker_pose_world = pose * marker_pose_base;
    marker.pose = tf2::toMsg( marker_pose_world );
    marker.header.frame_id = frame_id;
  }
  marker_array.markers.insert( marker_array.markers.end(), marker_array_temp.markers.begin(),
                               marker_array_temp.markers.end() );
}

void ShapeModel::getRobotShapeVisualization( visualization_msgs::msg::MarkerArray &marker_array,
                                             const Eigen::Isometry3d &pose, std::string frame_id,
                                             const Eigen::Vector3d &color ) const
{
  for ( unsigned int i = 0; i < getShape().size(); i++ ) {
    visualization_msgs::msg::Marker marker = getShape()[i]->getVisualizationMarker();
    marker.header.frame_id = frame_id;
    marker.ns = "robot_shape";
    marker.id = i;
    marker.color.r = color( 0 );
    marker.color.g = color( 1 );
    marker.color.b = color( 2 );

    Eigen::Isometry3d marker_pose = pose * getShape()[i]->getBaseTransform();
    marker.pose = tf2::toMsg( marker_pose );
    marker_array.markers.push_back( marker );
    const std::vector<Eigen::Vector3d> &sampling_points = getShape()[i]->getSamplingPoints();
    for ( unsigned int j = 0; j < sampling_points.size(); j++ ) {
      visualization_msgs::msg::Marker sp_marker;
      sp_marker.type = visualization_msgs::msg::Marker::SPHERE;
      sp_marker.action = visualization_msgs::msg::Marker::ADD;
      sp_marker.scale.x = 0.02;
      sp_marker.scale.y = 0.02;
      sp_marker.scale.z = 0.02;
      if ( getShape()[i]->isTrack() ) {
        sp_marker.color = collisionTypeToColor( TRACK );
      } else if ( getShape()[i]->isBody() ) {
        sp_marker.color = collisionTypeToColor( BODY );
      } else {
        sp_marker.color = collisionTypeToColor( DEFAULT );
      }

      sp_marker.header.frame_id = frame_id;
      sp_marker.ns = "sampling_points_shape_" + std::to_string( i );
      sp_marker.id = j;

      Eigen::Isometry3d sp_marker_pose = Eigen::Isometry3d::Identity();
      sp_marker_pose.translation() = sampling_points[j];
      sp_marker_pose = pose * sp_marker_pose;
      sp_marker.pose = tf2::toMsg( sp_marker_pose );
      marker_array.markers.push_back( sp_marker );
    }
  }
}

moveit_msgs::msg::DisplayRobotState
ShapeModel::getDisplayRobotStateMsg( const Eigen::Isometry3d &robot_pose ) const
{
  if ( !world_virtual_joint_ ) {
    return moveit_msgs::msg::DisplayRobotState();
  }
  moveit::core::RobotState state_copy( *robot_state_ );
  state_copy.setJointPositions( world_virtual_joint_, robot_pose );
  moveit_msgs::msg::DisplayRobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg( state_copy, robot_state_msg.state );
  return robot_state_msg;
}

hector_math::Vector3<double> ShapeModel::computeCenterOfMass() const
{
  robot_mass_ = 0.0;
  Eigen::Vector3d com( Eigen::Vector3d::Zero() );
  for ( auto const &link : robot_model_->getURDF()->links_ ) {
    if ( !link.second->inertial || link.second->inertial->mass <= 0 ) {
      continue;
    }
    double mass = link.second->inertial->mass;
    Eigen::Vector3d link_com;
    link_com.x() = link.second->inertial->origin.position.x;
    link_com.y() = link.second->inertial->origin.position.y;
    link_com.z() = link.second->inertial->origin.position.z;
    const Eigen::Isometry3d &transform = robot_state_->getGlobalLinkTransform( link.second->name );

    com += transform * link_com * mass;
    robot_mass_ += mass;
  }
  com /= robot_mass_;
  return com;
}

hector_math::Polygon<double> ShapeModel::computeFootprint() const
{
  RCLCPP_WARN_STREAM( node_->get_logger(), "ShapeModel::computeFootprint() is not implemented." );
  return {};
}

Eigen::AlignedBox<double, 3> ShapeModel::computeAxisAlignedBoundingBox() const
{
  RCLCPP_WARN_STREAM( node_->get_logger(),
                      "ShapeModel::computeAxisAlignedBoundingBox() is not implemented." );
  return {};
}

void ShapeModel::loadParameters( const rclcpp::Node::SharedPtr node )
{
  double default_resolution = node->declare_parameter<double>( "default_resolution", 0.1 );

  // --- which YAML file to read collision_links from? ---
  const std::string config_file =
      node->declare_parameter<std::string>( "collision_links_config_file", "" );

  if ( config_file.empty() ) {
    RCLCPP_ERROR( node->get_logger(), "Parameter 'collision_links_config_file' is empty. "
                                      "Cannot load collision_links." );
    return;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile( config_file );
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR_STREAM( node->get_logger(),
                         "Failed to load YAML config file '" << config_file << "': " << e.what() );
    return;
  }

  // --- collision_links array ---
  YAML::Node collision_link_info = root["collision_links"];
  if ( !collision_link_info || !collision_link_info.IsSequence() ) {
    RCLCPP_ERROR( node->get_logger(), "YAML: 'collision_links' is missing or not a list in '%s'.",
                  config_file.c_str() );
    return;
  }

  collision_links_.clear();
  collision_links_.reserve( collision_link_info.size() );

  for ( std::size_t i = 0; i < collision_link_info.size(); ++i ) {
    const YAML::Node &link_info = collision_link_info[i];
    if ( !link_info.IsMap() ) {
      RCLCPP_ERROR( node->get_logger(), "YAML: 'collision_links[%zu]' is not a map.", i );
      continue;
    }

    CollisionInfo info;

    // link (required)
    if ( link_info["link"] && link_info["link"].IsScalar() ) {
      info.link_name = link_info["link"].as<std::string>();
    } else {
      RCLCPP_ERROR( node->get_logger(),
                    "YAML: 'collision_links[%zu].link' is missing or not a string.", i );
      continue;
    }

    // type (optional, default "default")
    std::string type_str = "default";
    if ( link_info["type"] ) {
      type_str = link_info["type"].as<std::string>();
    }
    info.type = stringToCollisionType( type_str );

    // ignore_indices (optional, default empty)
    if ( link_info["ignore_indices"] && link_info["ignore_indices"].IsSequence() ) {
      info.ignore_indices = link_info["ignore_indices"].as<std::vector<int>>();
    }

    // include_indices (optional, default empty)
    if ( link_info["include_indices"] && link_info["include_indices"].IsSequence() ) {
      info.include_indices = link_info["include_indices"].as<std::vector<int>>();
    }

    // resolution (optional, default default_resolution)
    if ( link_info["resolution"] ) {
      info.sampling_info.resolution = link_info["resolution"].as<double>();
    } else {
      info.sampling_info.resolution = default_resolution;
    }

    // cylinder_angle_min/max (optional)
    if ( link_info["cylinder_angle_min"] ) {
      info.sampling_info.cylinder_angle_min = link_info["cylinder_angle_min"].as<double>();
    } else {
      info.sampling_info.cylinder_angle_min = 0.0;
    }

    if ( link_info["cylinder_angle_max"] ) {
      info.sampling_info.cylinder_angle_max = link_info["cylinder_angle_max"].as<double>();
    } else {
      info.sampling_info.cylinder_angle_max = 2.0 * M_PI;
    }

    collision_links_.push_back( std::move( info ) );
  }
}

void ShapeModel::loadRobotModel( const rclcpp::Node::SharedPtr node )
{
  // 1) URDF from /robot_description
  auto urdf_text = waitForStringMessage( node, "/robot_description", std::chrono::seconds( 5 ) );

  if ( !urdf_text ) {
    RCLCPP_ERROR( node->get_logger(), "Failed to load URDF from '/robot_description'." );
    return;
  }

  auto urdf = std::make_shared<urdf::Model>();
  if ( !urdf->initString( *urdf_text ) ) {
    RCLCPP_ERROR( node->get_logger(), "Failed to parse URDF from '/robot_description'." );
    return;
  }

  // 2) SRDF from /robot_description_semantic (optional)
  auto srdf_text =
      waitForStringMessage( node, "/robot_description_semantic", std::chrono::seconds( 5 ) );

  auto srdf = std::make_shared<srdf::Model>();
  if ( srdf_text && !srdf_text->empty() ) {
    if ( !srdf->initString( *urdf, *srdf_text ) ) {
      RCLCPP_WARN( node->get_logger(), "Failed to parse SRDF from '/robot_description_semantic'. "
                                       "Proceeding without SRDF." );
    }
  } else {
    RCLCPP_WARN( node->get_logger(), "No SRDF received on '/robot_description_semantic'. "
                                     "Proceeding without SRDF." );
  }

  // 3) Build MoveIt RobotModel + RobotState
  try {
    robot_model_ = std::make_shared<moveit::core::RobotModel>( urdf, srdf );
    robot_state_ = std::make_shared<moveit::core::RobotState>( robot_model_ );
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR_STREAM( node->get_logger(), "Failed to initialize robot model: " << e.what() );
    return;
  }

  robot_state_->setToDefaultValues();

  if ( robot_model_->hasJointModel( "world_virtual_joint" ) ) {
    world_virtual_joint_ = robot_model_->getJointModel( "world_virtual_joint" );
  } else {
    world_virtual_joint_ = nullptr;
  }

  // 4) Select active, non-mimic joint variables
  joint_names_.clear();
  for ( const std::string &variable : robot_model_->getVariableNames() ) {
    const auto *joint = robot_model_->getJointOfVariable( variable );
    if ( joint->getType() == moveit::core::JointModel::PRISMATIC ||
         joint->getType() == moveit::core::JointModel::REVOLUTE ||
         joint->getType() == moveit::core::JointModel::PLANAR ) {
      if ( !joint->isPassive() && !joint->getMimic() ) {
        joint_names_.push_back( variable );
      }
    }
  }

  // 5) Init joint positions from default state
  joint_positions_.clear();
  joint_positions_.reserve( joint_names_.size() );
  for ( const auto &joint_name : joint_names_ ) {
    joint_positions_.push_back( robot_state_->getVariablePosition( joint_name ) );
  }
}

void ShapeModel::generateShape()
{
  // Get collision shapes
  collision_bodies_.reserve( collision_links_.size() );
  for ( const CollisionInfo &info : collision_links_ ) {
    const moveit::core::LinkModel *link_model = robot_state_->getLinkModel( info.link_name );
    if ( link_model ) {
      if ( link_model->getShapes().empty() ) {
        RCLCPP_WARN_STREAM( node_->get_logger(),
                            "Link '" << info.link_name << "' does not have any collision geometry." );
      }
      for ( unsigned int i = 0; i < link_model->getShapes().size(); ++i ) {
        if ( !info.include_indices.empty() &&
             std::find( info.include_indices.begin(), info.include_indices.end(),
                        static_cast<int>( i ) ) == info.include_indices.end() ) {
          continue;
        }

        if ( !info.ignore_indices.empty() &&
             std::find( info.ignore_indices.begin(), info.ignore_indices.end(),
                        static_cast<int>( i ) ) != info.ignore_indices.end() ) {
          // Skip this index
          continue;
        }
        CollisionBody collision_body;
        collision_body.link_model_ptr = link_model;
        collision_body.index = i;
        collision_body.shape_ptr =
            convertShape( link_model->getShapes()[i], info.type, info.sampling_info );

        if ( collision_body.shape_ptr ) {
          collision_body.offset = collision_body.shape_ptr->getBaseTransform();
          shape_.push_back( collision_body.shape_ptr );
          collision_bodies_.push_back( std::move( collision_body ) );
          //          ROS_INFO_STREAM("Added shape " << info.link_name << ", " << i);
        } else {
          RCLCPP_WARN_STREAM( node_->get_logger(), "Could not convert shape " << i << " of link '"
                                                                              << info.link_name
                                                                              << "'." );
        }
      }
    } else {
      RCLCPP_ERROR_STREAM( node_->get_logger(), "Unknown link '" << info.link_name << "'" );
    }
  }
  // Collect total number of sample points
  total_sampling_point_count_ = 0;
  for ( const auto &shape : getShape() ) {
    total_sampling_point_count_ += shape->getSamplingPointsCount();
  }
  updateShape();
}

void ShapeModel::updateShape()
{
  // Update joint positions
  robot_state_->setVariablePositions( joint_names_, joint_positions_ );
  // Update transforms
  robot_state_->updateCollisionBodyTransforms();
  // Update shapes with transform
  for ( const CollisionBody &collision_body : collision_bodies_ ) {
    const Eigen::Isometry3d transform = robot_state_->getCollisionBodyTransform(
        collision_body.link_model_ptr, collision_body.index );
    collision_body.shape_ptr->setBaseTransform( transform * collision_body.offset );
  }
}

std::optional<std::string> ShapeModel::waitForStringMessage( const rclcpp::Node::SharedPtr &node,
                                                             const std::string &topic,
                                                             std::chrono::milliseconds timeout )
{
  using Msg = std_msgs::msg::String;

  std::promise<std::string> promise;
  auto future = promise.get_future();

  // Make sure we only set the promise once, without calling get_future() again
  auto received = std::make_shared<std::atomic_bool>( false );
  const auto qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).transient_local();
  // Keep subscription alive in this scope
  auto sub =
      node->create_subscription<Msg>( topic, qos, [received, &promise]( const Msg::SharedPtr msg ) {
        bool expected = false;
        if ( received->compare_exchange_strong( expected, true ) ) {
          // first time we see a message -> fulfill promise
          promise.set_value( msg->data );
        }
      } );

  // Spin this node locally until the future is ready or timeout happens
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node( node );

  auto ret = exec.spin_until_future_complete( future, timeout );

  exec.remove_node( node );

  if ( ret == rclcpp::FutureReturnCode::SUCCESS ) {
    return future.get();
  }

  RCLCPP_WARN( node->get_logger(), "Timeout while waiting for topic '%s', Node namespace: '%s'",
               topic.c_str(), node->get_namespace() );

  return std::nullopt;
}

ShapePtr ShapeModel::convertShape( const shapes::ShapeConstPtr &shape_ptr, CollisionType type,
                                   SamplingInfo sampling_info )
{
  if ( sampling_info.resolution < std::numeric_limits<double>::epsilon() ) {
    RCLCPP_ERROR_STREAM( rclcpp::get_logger( "shape_conversion" ),
                         "Resolution is zero or lower. Using 0.1 instead." );
    sampling_info.resolution = 0.1;
  }
  ShapePtr shape;
  switch ( shape_ptr->type ) {
  case shapes::ShapeType::BOX: {
    const auto *box = dynamic_cast<const shapes::Box *>( shape_ptr.get() );
    shape = convertBox( box, sampling_info );
    break;
  }
  case shapes::ShapeType::CYLINDER: {
    const auto *cylinder = dynamic_cast<const shapes::Cylinder *>( shape_ptr.get() );
    shape = convertCylinder( cylinder, sampling_info );
    break;
  }
  default:
    return {};
  }
  if ( shape ) {
    shape->setTrack( type == CollisionType::TRACK );
    shape->setBody( type == CollisionType::BODY );
  }
  return shape;
}

ShapePtr ShapeModel::convertBox( const shapes::Box *box, const SamplingInfo &sampling_info )
{
  Eigen::Isometry3d offset;
  offset = Eigen::Translation3d( 0, 0, -box->size[2] / 2.0 );
  ShapePtr shape =
      std::make_shared<RectangleShape>( box->size[0], box->size[1], offset, sampling_info );
  return shape;
}

ShapePtr ShapeModel::convertCylinder( const shapes::Cylinder *cylinder,
                                      const SamplingInfo &sampling_info )
{
  ShapePtr shape = std::make_shared<CylinderShape>( cylinder->radius, cylinder->length,
                                                    Eigen::Isometry3d::Identity(), sampling_info );
  return shape;
}

void ShapeModel::onJointStatesUpdated()
{
  RobotModel::onJointStatesUpdated();
  RCLCPP_DEBUG_STREAM( node_->get_logger(),
                       "[SDFContactEstimation::updateJointStates] Setting joint state to "
                           << vectorToString( joint_positions_ ) );
  updateShape();
}

const double &ShapeModel::mass() const
{
  if ( robot_mass_ <= 0.0 ) {
    computeCenterOfMass();
  }
  return robot_mass_;
}

} // namespace sdf_contact_estimation
