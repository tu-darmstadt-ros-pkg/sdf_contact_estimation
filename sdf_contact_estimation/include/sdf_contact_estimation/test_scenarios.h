#ifndef SDF_CONTACT_ESTIMATION_TEST_SCENARIOS_H
#define SDF_CONTACT_ESTIMATION_TEST_SCENARIOS_H

#include <pcl_ros/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

namespace sdf_contact_estimation {
  pcl::PointCloud<pcl::PointXYZ> generatePlane(double minX, double maxX, double minY, double maxY, double resolution);
  pcl::PointCloud<pcl::PointXYZ> generateCuboid(double length_x, double length_y, double length_z, double resolution);
  pcl::PointCloud<pcl::PointXYZ> generateFlat();
  pcl::PointCloud<pcl::PointXYZ> generateRamp();
  pcl::PointCloud<pcl::PointXYZ> generateStep(double step_height);
  pcl::PointCloud<pcl::PointXYZ> generateObstacle();
  pcl::PointCloud<pcl::PointXYZ> generateHole();

  std::string getScenarioName(const std::string& full_name);
  double getScenarioValue(const std::string& full_name, double default_value);
  pcl::PointCloud<pcl::PointXYZ> createScenarioFromName(std::string name);
}

#endif
