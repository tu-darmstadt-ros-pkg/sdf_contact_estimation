#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <sdf_contact_estimation/robot_model/shape_model.h>
#include <sdf_contact_estimation/util/timing.h>
#include <sdf_contact_estimation/sdf/sdf_model.h>
#include <sdf_contact_estimation/util/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>

static bool pose_updated_;
static Eigen::Isometry3d robot_pose_;
static bool joint_state_updated_;
static std::vector<std::string> active_joint_names_;
static std::vector<double> joint_state_;
static std::vector<double> previous_state_;

void poseCB(const geometry_msgs::PoseStamped& pose_msg) {
  tf::poseMsgToEigen(pose_msg.pose, robot_pose_);
  pose_updated_ = true;
}

void jointStateCb(const sensor_msgs::JointStateConstPtr& joint_state_msg) {
  bool state_changed = false;
  for (unsigned int joint_idx = 0; joint_idx < active_joint_names_.size(); ++joint_idx) {
    auto it = std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), active_joint_names_[joint_idx]);
    if (it != joint_state_msg->name.end()) {
      size_t index = static_cast<size_t>(it - joint_state_msg->name.begin());
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "sdf_contact_estimation");
  ros::NodeHandle pnh("~");

  // Robot pose
  std::vector<double> robot_pose_vec = pnh.param<std::vector<double>>("robot_pose", std::vector<double>(6, 0));
  if (robot_pose_vec.size() != 6) {
    ROS_WARN_STREAM("Robot pose vector has wrong size (" << robot_pose_vec.size() << ") instead of 6. Using identity.");
    robot_pose_vec.resize(6, 0);
  }

  robot_pose_ = Eigen::AngleAxisd(robot_pose_vec[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(robot_pose_vec[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(robot_pose_vec[3], Eigen::Vector3d::UnitX());
  robot_pose_.translation() = Eigen::Vector3d(robot_pose_vec[0], robot_pose_vec[1], robot_pose_vec[2]);
  pose_updated_ = true;
  ros::Subscriber robot_pose_sub = pnh.subscribe("set_robot_pose", 10, &poseCB);


  // Joint positions
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(pnh);
  active_joint_names_ = shape_model->jointNames();
  ROS_INFO_STREAM("Loaded joints: " << sdf_contact_estimation::vectorToString(active_joint_names_));
  joint_state_ = std::vector<double>(shape_model->jointNames().size(), 0);
  std::map<std::string, double> joint_state_map;
  if (pnh.getParam("/zeros", joint_state_map)) {
    for (const auto& joint_state: joint_state_map) {
      auto it = std::find(active_joint_names_.begin(), active_joint_names_.end(), joint_state.first);
      if (it != active_joint_names_.end()) {
        auto index = static_cast<size_t>(it - active_joint_names_.begin());
        joint_state_[index] = joint_state.second;
      } else {
        ROS_ERROR_STREAM("Unknown joint '" << joint_state.first);
      }
    }
  }
  joint_state_updated_ = true;
  previous_state_ = joint_state_;
  ros::Subscriber flipper_angle_sub = pnh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &jointStateCb);

  // SDF
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(pnh);
  ros::NodeHandle sdf_model_nh(pnh, "sdf_map");
  sdf_model->loadFromServer(sdf_model_nh);

  // Pose Predictor
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(pnh, shape_model, sdf_model);
  sdf_pose_predictor->enableVisualisation(true);
  auto pose_predictor = std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    if (pose_updated_ || joint_state_updated_) {
      pose_predictor->robotModel()->updateJointPositions(joint_state_);
      hector_math::Pose<double> robot_pose(robot_pose_);
      pose_predictor->predictPose(robot_pose);
      pose_updated_ = false;
      joint_state_updated_ = false;
    }
    rate.sleep();
  }

  return 0;
}
