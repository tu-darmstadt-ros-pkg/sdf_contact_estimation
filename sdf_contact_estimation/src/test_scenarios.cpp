#include <sdf_contact_estimation/test_scenarios.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

namespace sdf_contact_estimation {
  pcl::PointCloud<pcl::PointXYZ> generatePlane(double minX, double maxX, double minY, double maxY, double resolution) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (double x = minX; x <= maxX; x += resolution) {
      for (double y = minY; y <= maxY; y += resolution) {
        cloud.push_back(pcl::PointXYZ(x, y, 0));
      }
    }

    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> generateCuboid(double length_x, double length_y, double length_z, double resolution) {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    double min_x = -length_x/2.0;
    double max_x = -min_x;
    double min_y = -length_y/2.0;
    double max_y = -min_y;
    double min_z = -length_z/2.0;
    double max_z = -min_z;

    for(double x = min_x; x <= max_x; x += resolution) {
      for(double y = min_y; y <= max_y; y += resolution) {
        double z = min_z;
        cloud.push_back(pcl::PointXYZ(x, y, z));
        z = max_z;
        cloud.push_back(pcl::PointXYZ(x, y, z));
      }
    }
    for(double x = min_x; x <= max_x; x += resolution) {
      for(double z = min_z; z <= max_z; z += resolution) {
        double y = min_y;
        cloud.push_back(pcl::PointXYZ(x, y, z));
        y = max_y;
        cloud.push_back(pcl::PointXYZ(x, y, z));
      }
    }

    for(double y = min_y; y <= max_y; y += resolution) {
      for(double z = min_z; z <= max_z; z += resolution) {
        double x = min_x;
        cloud.push_back(pcl::PointXYZ(x, y, z));
        x = max_x;
        cloud.push_back(pcl::PointXYZ(x, y, z));
      }
    }
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> generateFlat() {
    auto plane = generatePlane(-1, 1, -1, 1, 0.025);
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translation() = Eigen::Vector3d(0, 0, -1);
    pcl::transformPointCloud(plane, plane, t);
    return plane;
  }

  pcl::PointCloud<pcl::PointXYZ> generateRamp() {
    Eigen::Affine3d tilt(Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitY()));
    tilt.translation() = Eigen::Vector3d(0, 0, -0.8);

    auto ramp = generatePlane(-0.5, 0.5, -1, 1, 0.025);
    pcl::transformPointCloud(ramp, ramp, tilt);

    auto upper_plane = generatePlane(0.5, 2, -1, 1, 0.025);
    Eigen::Affine3d t;
    t = Eigen::Translation3d(-0.05, 0, -0.6);
    pcl::transformPointCloud(upper_plane, upper_plane, t);

    auto lower_plane = generatePlane(-2, -0.5, -1, 1, 0.025);
    t = Eigen::Translation3d(0.05, 0, -1);
    pcl::transformPointCloud(lower_plane, lower_plane, t);

    auto merged = ramp + upper_plane;
    merged = merged + lower_plane;
    return merged;
  }

  pcl::PointCloud<pcl::PointXYZ> generateStep(double step_height) {
    const double resolution = 0.025;
    const double z_offset = -2;
    const double x_offset = 1;
    auto plane = generatePlane(-2, 0, -1, 1, resolution);
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translation() = Eigen::Vector3d(x_offset, 0, z_offset);
    pcl::transformPointCloud(plane, plane, t);

    auto step = generatePlane(0, 2, -1, 1, resolution);
    t.translation() = Eigen::Vector3d(x_offset, 0, z_offset + step_height);
    pcl::transformPointCloud(step, step, t);

    auto step_side = generatePlane(-step_height, 0, -1, 1, resolution);
    Eigen::Affine3d t2(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()));
    t2.translation() = Eigen::Vector3d(x_offset, 0, z_offset + step_height);
    pcl::transformPointCloud(step_side, step_side, t2);


    pcl::PointCloud<pcl::PointXYZ> merged = plane + step;
    merged = merged + step_side;

    return merged;
  }

  pcl::PointCloud<pcl::PointXYZ> generateObstacle() {
    auto plane = generatePlane(-1, 0, -1, 1, 0.025);
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translation() = Eigen::Vector3d(0, 0, -1);
    pcl::transformPointCloud(plane, plane, t);

    auto step = generatePlane(0, 1, -1, -0.2, 0.025);
    t.translation() = Eigen::Vector3d(0, 0, -0.9);
    pcl::transformPointCloud(step, step, t);

    auto plane2 = generatePlane(0, 1, -0.2, 1, 0.025);
    t.translation() = Eigen::Vector3d(0, 0, -1);
    pcl::transformPointCloud(plane2, plane2, t);

    pcl::PointCloud<pcl::PointXYZ> merged = plane + plane2;
    merged += step;

    return merged;
  }

  pcl::PointCloud<pcl::PointXYZ> generateHole() {
    auto plane = generatePlane(-1, 1, -1, 1, 0.025);
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translation() = Eigen::Vector3d(0, 0, -1);
    pcl::transformPointCloud(plane, plane, t);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(plane, *plane_ptr);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(plane_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.05, 0.05);
    pass.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*x_filtered);

    pass.setInputCloud(x_filtered);
    pass.setFilterFieldName("y");
    pcl::PointCloud<pcl::PointXYZ>::Ptr y_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*y_filtered);

    return *y_filtered;
  }

  std::string getScenarioName(const std::string& full_name)
  {
    size_t pos;
    if ((pos = full_name.find("_")) != std::string::npos) {
      return full_name.substr(0, pos);
    } else {
      return full_name;
    }
  }

  double getScenarioValue(const std::string& full_name, double default_value)
  {
    const std::string delimiter = "_";
    size_t pos;
    if ((pos = full_name.find(delimiter)) != std::string::npos) {
      std::string value_str = full_name.substr(pos + delimiter.length(), full_name.length());

      try {
        return std::stod(value_str);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to convert '" << value_str << "' to a double. Using default " << default_value);
        return default_value;
      }
    } else {
      return default_value;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> createScenarioFromName(std::string name)
  {
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    std::string scenario_name = getScenarioName(name);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (scenario_name == "flat") {
      cloud = sdf_contact_estimation::generateFlat();
    } else if (scenario_name == "ramp") {
      cloud = sdf_contact_estimation::generateRamp();
    } else if (scenario_name == "step") {
      double height = getScenarioValue(name, 0.18);
      cloud = sdf_contact_estimation::generateStep(height);
    } else if (scenario_name == "obstacle") {
      cloud = sdf_contact_estimation::generateObstacle();
    } else if (scenario_name == "hole") {
      cloud = sdf_contact_estimation::generateHole();
    } else {
      ROS_ERROR_STREAM("Unknown scenario '" << name << "'.");
      cloud = sdf_contact_estimation::generateFlat();
    }
    cloud.header.frame_id = "world";
    return cloud;
  }
}
