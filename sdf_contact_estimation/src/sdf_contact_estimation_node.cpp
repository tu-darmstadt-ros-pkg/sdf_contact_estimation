#include <algorithm>
#include <map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/util/timing.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/util/utils.h>
#include <Eigen/Geometry>

static bool pose_updated_;
static Eigen::Isometry3d robot_pose_;
static bool joint_state_updated_;
static std::vector<std::string> active_joint_names_;
static std::vector<double> joint_state_;
static std::vector<double> previous_state_;

// Pose callback (ROS 2)
void poseCB(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
  tf2::fromMsg(pose_msg->pose, robot_pose_);
  pose_updated_ = true;
}

// Joint state callback (ROS 2)
void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
{
  bool state_changed = false;
  for (std::size_t joint_idx = 0; joint_idx < active_joint_names_.size(); ++joint_idx) {
    auto it = std::find(joint_state_msg->name.begin(),
                        joint_state_msg->name.end(),
                        active_joint_names_[joint_idx]);
    if (it != joint_state_msg->name.end()) {
      const std::size_t index = static_cast<std::size_t>(it - joint_state_msg->name.begin());
      joint_state_[joint_idx] = joint_state_msg->position[index];

      if (std::abs(joint_state_[joint_idx] - previous_state_[joint_idx]) > 0.01) {
        state_changed = true;
      }
    }
  }

  if (state_changed) {
    joint_state_updated_ = true;
    previous_state_ = joint_state_;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sdf_contact_estimation");

 // sleep for a short time to allow other nodes to start
  rclcpp::sleep_for(std::chrono::seconds(10));

  // --- Robot pose parameter -------------------------------------------------
  // Same semantics as ROS1: 6-dim vector [x, y, z, roll, pitch, yaw]
  std::vector<double> default_pose(6, 0.0);
  auto robot_pose_vec =
      node->declare_parameter<std::vector<double>>("robot_pose", default_pose);

  if (robot_pose_vec.size() != 6) {
    RCLCPP_WARN(
      node->get_logger(),
      "Robot pose vector has wrong size (%zu) instead of 6. Using identity.",
      robot_pose_vec.size());
    robot_pose_vec.assign(6, 0.0);
  }

  robot_pose_ = Eigen::AngleAxisd(robot_pose_vec[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(robot_pose_vec[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(robot_pose_vec[3], Eigen::Vector3d::UnitX());
  robot_pose_.translation() = Eigen::Vector3d(robot_pose_vec[0], robot_pose_vec[1], robot_pose_vec[2]);
  pose_updated_ = true;

  // Pose subscriber (set_robot_pose)
  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "set_robot_pose", 10, &poseCB);

  // --- Shape model ----------------------------------------------------------
  // In ROS1: NodeHandle shape_model_nh(pnh, "shape_model");
  // Here we use a sub-node so parameters can be namespaced under "shape_model".
  auto shape_model_node = node->create_sub_node("shape_model");
  auto shape_model =
      std::make_shared<sdf_contact_estimation::ShapeModel>(node);

  active_joint_names_ = shape_model->jointNames();
  RCLCPP_INFO(
      node->get_logger(), "Loaded joints: %s",
      sdf_contact_estimation::vectorToString(active_joint_names_).c_str());

  joint_state_.assign(active_joint_names_.size(), 0.0);
  previous_state_ = joint_state_;
  joint_state_updated_ = true;

  // --- Optional "zeros" parameter (initial joint positions) -----------------
  // ROS1 had: pnh.getParam("/zeros", std::map<string,double>)
  //
  // ROS2 version: use parameters with prefix "zeros".
  // Expect parameters like:
  //   zeros.flipper_front_joint: 0.0
  //   zeros.flipper_back_joint:  0.0
  //
  // The key after "zeros." is interpreted as the joint name.
  std::map<std::string, double> joint_state_map;
  if (node->get_parameters("zeros", joint_state_map)) {
    for (const auto &entry : joint_state_map) {
      const std::string &joint_name = entry.first;
      double value = entry.second;

      auto it = std::find(
          active_joint_names_.begin(), active_joint_names_.end(), joint_name);
      if (it != active_joint_names_.end()) {
        std::size_t idx =
            static_cast<std::size_t>(it - active_joint_names_.begin());
        joint_state_[idx] = value;
      } else {
        RCLCPP_ERROR(
            node->get_logger(),
            "Unknown joint '%s' in zeros parameter, ignoring.",
            joint_name.c_str());
      }
    }
    previous_state_ = joint_state_;
  }

  // --- Joint state subscriber -----------------------------------------------
  auto joint_state_sub =
      node->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10, &jointStateCb);

  // --- SDF model ------------------------------------------------------------
  // In ROS1: NodeHandle sdf_model_nh(pnh, "sdf_map");
  auto sdf_model_node = node->create_sub_node("sdf_map");
  auto sdf_model =
      std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_node);
  sdf_model->loadFromServer(sdf_model_node);

  // --- Pose predictor -------------------------------------------------------
  auto sdf_pose_predictor =
      std::make_shared<sdf_contact_estimation::SDFContactEstimation>(
          node, shape_model, sdf_model);
  sdf_pose_predictor->enableVisualisation(true);

  std::shared_ptr<hector_pose_prediction_interface::PosePredictor<double>>
      pose_predictor = sdf_pose_predictor;

  // --- Main loop ------------------------------------------------------------
  rclcpp::Rate rate(100.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (pose_updated_ || joint_state_updated_) {
      // Update robot model joint positions
      pose_predictor->robotModel()->updateJointPositions(joint_state_);

      hector_math::Pose<double> hector_pose(robot_pose_);
      pose_predictor->predictPose(hector_pose);

      pose_updated_ = false;
      joint_state_updated_ = false;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
