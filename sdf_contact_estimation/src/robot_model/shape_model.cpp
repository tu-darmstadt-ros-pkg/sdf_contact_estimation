#include <sdf_contact_estimation/robot_model/shape_model.h>

#include <sdf_contact_estimation/robot_model/basic_shapes/rectangle_shape.h>
#include <sdf_contact_estimation/robot_model/basic_shapes/cylinder_shape.h>
#include <sdf_contact_estimation/util/utils.h>

#include <geometric_shapes/bodies.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

namespace sdf_contact_estimation {

ShapeModel::ShapeModel(const ros::NodeHandle& pnh)
: RobotModel(std::unordered_map<std::string, double>()), total_sampling_point_count_(0), robot_mass_(0.0)
{
  loadParameters(pnh);
  loadRobotModel(pnh);
  generateShape();
}

const RobotShape& ShapeModel::getShape() const {
  return shape_;
}

size_t ShapeModel::getTotalSamplingPointCount() const
{
  return total_sampling_point_count_;
}

void ShapeModel::getRobotStateVisualization(visualization_msgs::MarkerArray& marker_array, const Eigen::Isometry3d& pose, const std::string& frame_id) const
{
  std_msgs::ColorRGBA color;
  color.a = 0.8;
  color.b = 1.0;
  visualization_msgs::MarkerArray marker_array_temp;
  robot_state_->getRobotMarkers(marker_array_temp, robot_model_->getLinkModelNames(), color, "robot_state", ros::Duration(0));
  // Transform from base to world frame
  for (auto& marker: marker_array_temp.markers) {
    Eigen::Isometry3d marker_pose_base;
    tf::poseMsgToEigen(marker.pose, marker_pose_base);
    Eigen::Isometry3d marker_pose_world = pose * marker_pose_base;
    tf::poseEigenToMsg(marker_pose_world, marker.pose);
    marker.header.frame_id = frame_id;
  }
  marker_array.markers.insert(marker_array.markers.end(), marker_array_temp.markers.begin(), marker_array_temp.markers.end());
}

void ShapeModel::getRobotShapeVisualization(visualization_msgs::MarkerArray& marker_array, const Eigen::Isometry3d& pose,
                                            std::string frame_id, const Eigen::Vector3d& color) const
{
  for (unsigned int i = 0; i < getShape().size(); i++) {
    visualization_msgs::Marker marker = getShape()[i]->getVisualizationMarker();
    marker.header.frame_id = frame_id;
    marker.ns = "robot_shape";
    marker.id = i;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);

    Eigen::Isometry3d marker_pose = pose * getShape()[i]->getBaseTransform();
    tf::poseEigenToMsg(marker_pose, marker.pose);
    marker_array.markers.push_back(marker);
    const std::vector<Eigen::Vector3d>& sampling_points = getShape()[i]->getSamplingPoints();
    for (unsigned int j = 0; j < sampling_points.size(); j++) {
      visualization_msgs::Marker sp_marker;
      sp_marker.type = visualization_msgs::Marker::SPHERE;
      sp_marker.action = visualization_msgs::Marker::ADD;
      sp_marker.scale.x = 0.02;
      sp_marker.scale.y = 0.02;
      sp_marker.scale.z = 0.02;
      if (getShape()[i]->isTrack()) {
        sp_marker.color = collisionTypeToColor(TRACK);
      } else if (getShape()[i]->isBody()) {
        sp_marker.color = collisionTypeToColor(BODY);
      } else {
        sp_marker.color = collisionTypeToColor(DEFAULT);
      }

      sp_marker.header.frame_id = frame_id;
      sp_marker.ns = "sampling_points_shape_" + std::to_string(i);
      sp_marker.id = j;

      Eigen::Isometry3d sp_marker_pose = Eigen::Isometry3d::Identity();
      sp_marker_pose.translation() = sampling_points[j];
      sp_marker_pose = pose * sp_marker_pose;
      tf::poseEigenToMsg(sp_marker_pose, sp_marker.pose);
      marker_array.markers.push_back(sp_marker);
    }
  }
}

moveit_msgs::DisplayRobotState ShapeModel::getDisplayRobotStateMsg(const Eigen::Isometry3d& robot_pose) const
{
  if (!world_virtual_joint_) {
    return {};
  }
  robot_state::RobotState state_copy(*robot_state_);
  state_copy.setJointPositions(world_virtual_joint_, robot_pose);
  moveit_msgs::DisplayRobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(state_copy, robot_state_msg.state);
  return robot_state_msg;
}


hector_math::Vector3<double> ShapeModel::computeCenterOfMass() const {
  robot_mass_ = 0.0;
  Eigen::Vector3d com(Eigen::Vector3d::Zero());
  for (auto const& link: robot_model_->getURDF()->links_) {
    if (!link.second->inertial || link.second->inertial->mass <= 0) {
      continue;
    }
    double mass = link.second->inertial->mass;
    Eigen::Vector3d link_com;
    link_com.x() = link.second->inertial->origin.position.x;
    link_com.y() = link.second->inertial->origin.position.y;
    link_com.z() = link.second->inertial->origin.position.z;
    const Eigen::Isometry3d& transform = robot_state_->getGlobalLinkTransform(link.second->name);

    com += transform * link_com * mass;
    robot_mass_ += mass;
  }
  com /= robot_mass_;
  return com;
}

hector_math::Polygon<double> ShapeModel::computeFootprint() const {
  ROS_WARN_STREAM("ShapeModel::computeFootprint() is not implemented.");
  return {};
}

Eigen::AlignedBox<double, 3> ShapeModel::computeAxisAlignedBoundingBox() const {
  ROS_WARN_STREAM("ShapeModel::computeAxisAlignedBoundingBox() is not implemented.");
  return {};
}

void ShapeModel::loadParameters(const ros::NodeHandle& nh) {
  double default_resolution;
  nh.param("default_resolution", default_resolution, 0.1);
//  nh.param("joints", joint_names_, std::vector<std::string>());

  // Get collision link info
  XmlRpc::XmlRpcValue collision_link_info;
  if (!nh.getParam("collision_links", collision_link_info)) {
    ROS_ERROR_STREAM("Missing required parameter '" << nh.getNamespace() << "/collision_links");
    return;
  }
  if (collision_link_info.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("Parameter is not a list");
    return;
  }
  for(unsigned int i = 0; i < collision_link_info.size(); ++i) {
    const XmlRpc::XmlRpcValue& link_info = collision_link_info[i];
    if (link_info.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM("Parameter is not a struct");
      continue;
    }

    CollisionInfo info;

    if (link_info.hasMember("link") && link_info["link"].getType() == XmlRpc::XmlRpcValue::TypeString) {
      info.link_name = static_cast<std::string>(link_info["link"]);
    } else {
      ROS_ERROR_STREAM("Link name is missing.");
      continue;
    }

    std::string type_str = getXmlRpcValueWithDefault<std::string>(link_info, "type", "default");
    info.type = stringToCollisionType(type_str);

    info.ignore_indices = getXmlRpcValueWithDefault(link_info, "ignore_indices", std::vector<int>());
    info.include_indices = getXmlRpcValueWithDefault(link_info, "include_indices", std::vector<int>());

    info.sampling_info.resolution = getXmlRpcValueWithDefault(link_info, "resolution", default_resolution);
    info.sampling_info.cylinder_angle_min = getXmlRpcValueWithDefault(link_info, "cylinder_angle_min", 0.0);
    info.sampling_info.cylinder_angle_max = getXmlRpcValueWithDefault(link_info, "cylinder_angle_max", 2 * M_PI);


    collision_links_.push_back(std::move(info));
  }
}

void ShapeModel::loadRobotModel(const ros::NodeHandle& nh) {
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initParamWithNodeHandle("/robot_description", nh)) {
    ROS_ERROR("Failed to load urdf");
  }
  auto srdf = std::make_shared<srdf::Model>();
  std::string semantic_description;
  if (nh.getParam("/robot_description_semantic", semantic_description)) {
    srdf->initString(*urdf, semantic_description);
  }

  try {
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM( "Failed to initialize robot model: " << e.what());
  }
  robot_state_->setToDefaultValues();

  if (robot_model_->hasJointModel("world_virtual_joint")) {
    world_virtual_joint_ = robot_model_->getJointModel("world_virtual_joint");
  } else {
    world_virtual_joint_ = nullptr;
  }

  // Find joint variables, that are active, not a mimic and not floating (to filter out the world virtual joint)
  // If we find a way to identify virtual joints, we can also use floating joints
  for (const std::string& variable: robot_model_->getVariableNames()) {
    const moveit::core::JointModel* joint = robot_model_->getJointOfVariable(variable);
    if (joint->getType() == moveit::core::JointModel::JointType::PRISMATIC ||
        joint->getType() == moveit::core::JointModel::JointType::REVOLUTE ||
        joint->getType() == moveit::core::JointModel::JointType::PLANAR) {
      if (!joint->isPassive() && !joint->getMimic()) {
        joint_names_.push_back(variable);
      }
    }
  }

  // Init joint positions from default state
  joint_positions_.reserve(joint_names_.size());
  for (const std::string& joint_name: joint_names_) {
    joint_positions_.push_back(robot_state_->getVariablePosition(joint_name));
  }
}

void ShapeModel::generateShape() {
  // Get collision shapes
  collision_bodies_.reserve(collision_links_.size());
  for (const CollisionInfo& info: collision_links_) {
    const moveit::core::LinkModel* link_model = robot_state_->getLinkModel(info.link_name);
    if (link_model) {
      if (link_model->getShapes().empty()) {
        ROS_WARN_STREAM("Link '" << info.link_name << "' does not have any collision geometry.");
      }
      for (unsigned int i = 0; i < link_model->getShapes().size(); ++i) {
        if (!info.include_indices.empty() &&
            std::find(info.include_indices.begin(), info.include_indices.end(), static_cast<int>(i)) == info.include_indices.end()) {
          continue;
        }

        if (!info.ignore_indices.empty() &&
            std::find(info.ignore_indices.begin(), info.ignore_indices.end(), static_cast<int>(i)) != info.ignore_indices.end()) {
          // Skip this index
          continue;
        }
        CollisionBody collision_body;
        collision_body.link_model_ptr = link_model;
        collision_body.index = i;
        collision_body.shape_ptr = convertShape(link_model->getShapes()[i], info.type, info.sampling_info);

        if (collision_body.shape_ptr) {
          collision_body.offset = collision_body.shape_ptr->getBaseTransform();
          shape_.push_back(collision_body.shape_ptr);
          collision_bodies_.push_back(std::move(collision_body));
//          ROS_INFO_STREAM("Added shape " << info.link_name << ", " << i);
        } else {
          ROS_WARN_STREAM("Could not convert shape " << i << " of link '" << info.link_name << "'.");
        }
      }
    } else {
      ROS_ERROR_STREAM("Unknown link '" << info.link_name << "'");
    }
  }
  // Collect total number of sample points
  total_sampling_point_count_ = 0;
  for (const auto& shape: getShape()) {
    total_sampling_point_count_ += shape->getSamplingPointsCount();
  }
  updateShape();
}

void ShapeModel::updateShape() {
  // Update joint positions
  robot_state_->setVariablePositions(joint_names_, joint_positions_);
  // Update transforms
  robot_state_->updateCollisionBodyTransforms();
  // Update shapes with transform
  for (const CollisionBody& collision_body: collision_bodies_) {
    const Eigen::Isometry3d transform = robot_state_->getCollisionBodyTransform(collision_body.link_model_ptr, collision_body.index);
    collision_body.shape_ptr->setBaseTransform(transform * collision_body.offset);
  }
}

ShapePtr ShapeModel::convertShape(const shapes::ShapeConstPtr& shape_ptr, CollisionType type, SamplingInfo sampling_info)
{
  if (sampling_info.resolution < std::numeric_limits<double>::epsilon()) {
    ROS_ERROR_STREAM("Resolution is zero or lower. Using 0.1 instead.");
    sampling_info.resolution = 0.1;
  }
  ShapePtr shape;
  switch (shape_ptr->type) {
    case shapes::ShapeType::BOX: {
      const auto* box = dynamic_cast<const shapes::Box*>(shape_ptr.get());
      shape = convertBox(box, sampling_info);
      break;
    }
    case shapes::ShapeType::CYLINDER: {
      const auto* cylinder = dynamic_cast<const shapes::Cylinder*>(shape_ptr.get());
      shape = convertCylinder(cylinder, sampling_info);
      break;
    }
    default: return {};
  }
  if (shape) {
    shape->setTrack(type == CollisionType::TRACK);
    shape->setBody(type == CollisionType::BODY);
  }
  return shape;
}

ShapePtr ShapeModel::convertBox(const shapes::Box *box, const SamplingInfo& sampling_info)
{
  Eigen::Isometry3d offset; offset = Eigen::Translation3d(0, 0, -box->size[2]/2.0);
  ShapePtr shape = std::make_shared<RectangleShape>(
      box->size[0], box->size[1], offset, sampling_info);
  return shape;
}

ShapePtr ShapeModel::convertCylinder(const shapes::Cylinder *cylinder, const SamplingInfo& sampling_info)
{
  ShapePtr shape = std::make_shared<CylinderShape>(
      cylinder->radius, cylinder->length, Eigen::Isometry3d::Identity(), sampling_info);
  return shape;
}

void ShapeModel::onJointStatesUpdated()
{
  RobotModel::onJointStatesUpdated();
  ROS_DEBUG_STREAM("[SDFContactEstimation::updateJointStates] Setting joint state to " << vectorToString(joint_positions_));
  updateShape();
}

const double& ShapeModel::mass() const
{
  if (robot_mass_ <= 0.0) {
    computeCenterOfMass();
  }
  return robot_mass_;
}

}


