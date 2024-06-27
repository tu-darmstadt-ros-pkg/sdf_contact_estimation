#include <ros/ros.h>
#include <sdf_contact_estimation/test_scenarios.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>

// Parameters
std::string frame_id_;
double resolution_;
double track_height_offset_;
double footprint_min_x_, footprint_max_x_, footprint_min_y_, footprint_max_y_;
double subscriber_wait_timeout_;
double latching_time_;

bool loadParameters(const ros::NodeHandle& nh) {
  nh.param<std::string>("frame_id", frame_id_, "base_link");
  nh.param<double>("resolution", resolution_, 0.05);
  nh.param<double>("track_height_offset", track_height_offset_, -0.3);
  nh.param<double>("footprint_min_x", footprint_min_x_, -0.3);
  nh.param<double>("footprint_max_x", footprint_max_x_, 0.3);
  nh.param<double>("footprint_min_y", footprint_min_y_, -0.3);
  nh.param<double>("footprint_max_y", footprint_max_y_, 0.3);
  nh.param<double>("subscriber_wait_timeout", subscriber_wait_timeout_, 30.0);
  nh.param<double>("latching_time", latching_time_, 3.0);
  return true;
}

sensor_msgs::PointCloud2 createPlaneCloud(double min_x, double max_x, double min_y, double max_y, double z, double resolution, const std::string& frame_id) {
  pcl::PointCloud<pcl::PointXYZ> plane_cloud = sdf_contact_estimation::generatePlane(min_x, max_x, min_y, max_y, resolution);

  Eigen::Affine3d z_transform(Eigen::Translation3d(0.0, 0.0, z));
  pcl::PointCloud<pcl::PointXYZ> plane_cloud_transformed;
  pcl::transformPointCloud(plane_cloud, plane_cloud_transformed, z_transform);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(plane_cloud_transformed, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.header.stamp = ros::Time::now();
  return cloud_msg;
}

void waitForClock() {
  ros::spinOnce();
  ros::Rate rate(10.0);
  while (ros::ok() && ros::Time::now() == ros::Time(0)) {
    rate.sleep();
    ros::spinOnce();
  }
}

bool waitForSubscribers(const ros::Publisher& pub, double timeout) {
  ros::Duration timeout_subscribers(timeout);
  ros::Time wait_start = ros::Time::now();
  ros::Rate rate(10.0);
  while (pub.getNumSubscribers() < 1 &&
         (timeout_subscribers <= ros::Duration(0) || ros::Time::now() < wait_start + timeout_subscribers) &&
         ros::ok()
  ) {
    rate.sleep();
    ros::spinOnce();
  }
  return pub.getNumSubscribers() > 0;
}

void spin(double timeout) {
  ros::Rate rate(10.0);
  ros::Duration timeout_duration(timeout);
  ros::Time wait_start = ros::Time::now();
  while ((latching_time_ <= 0.0 || ros::Time::now() < wait_start + timeout_duration) && ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_plane_publisher");
  ros::NodeHandle pnh("~");

  waitForClock();

  if (!loadParameters(pnh)) {
    return 1;
  }

  ros::Publisher cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("cloud", 10, true);
  ROS_INFO_STREAM("Waiting for subscribers on " << cloud_pub.getTopic() <<  (subscriber_wait_timeout_ > 0.0 ? " for " + std::to_string(subscriber_wait_timeout_) + " s." : " until shutdown."));
  if (!waitForSubscribers(cloud_pub, subscriber_wait_timeout_)) {
    ROS_WARN_STREAM("No subscriber found on " << cloud_pub.getTopic());
  }

  sensor_msgs::PointCloud2 cloud_msg = createPlaneCloud(footprint_min_x_, footprint_max_x_, footprint_min_y_, footprint_max_y_, track_height_offset_, resolution_, frame_id_);
  for (unsigned int i = 0; i < 100; i++) {
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pub.publish(cloud_msg);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO_STREAM("Latching the ground plane cloud on " << cloud_pub.getTopic() <<  (latching_time_ > 0.0 ? " for " + std::to_string(latching_time_) + " s." : " until shutdown."));
  spin(latching_time_);

  return 0;
}