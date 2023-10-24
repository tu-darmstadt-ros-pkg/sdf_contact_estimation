#include <sdf_contact_estimation/util/utils.h>

namespace sdf_contact_estimation {

Eigen::Quaterniond rpyToRot(double roll, double pitch, double yaw) {
  Eigen::Quaterniond quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  return quat;
}

Eigen::Vector3d rotToNormalizedRpy(const Eigen::Matrix3d &rot) {
  double epsilon = 1e-12;
  double roll, pitch, yaw;
  pitch = std::atan2(-rot.data()[2], sqrt(rot.data()[0]*rot.data()[0] + rot.data()[1]*rot.data()[1]));
  if (std::abs(pitch) > (M_PI_2-epsilon)) {
    yaw = std::atan2(-rot.data()[3], rot.data()[4]);
    roll = 0.0;
  } else {
    roll = std::atan2(rot.data()[5], rot.data()[8]);
    yaw = std::atan2(rot.data()[1], rot.data()[0]);
  }
  return Eigen::Vector3d(roll, pitch, yaw);
}

double distanceToLine(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &x) {
  return (x-p1).cross(x-p2).norm()/ (p2 - p1).norm();
}

bool isOnLine(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &x) {
  double distance_to_line = distanceToLine(p1, p2, x);
  double threshold = 0.02;
  return distance_to_line < threshold;
}

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0, -v(2), v(1),
      v(2), 0, -v(0),
      -v(1), v(0), 0;
  return m;
}

Eigen::Matrix3d computeGravityAlignedRotationFromTo(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
  // Align without z first so q is gravity aligned
  Eigen::Vector3d v2_no_z = v2;
  v2_no_z.z() = 0;

  Eigen::Quaterniond q;
  q.setFromTwoVectors(v1, v2_no_z);

  // Now rotate around y to also align with z value
  Eigen::Quaterniond q2;
  q2.setFromTwoVectors(v2_no_z, v2);

  return Eigen::Matrix3d(q2 * q);
}

double distanceToLine2D(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& x)
{
  double x0 = x(0);
  double y0 = x(1);
  double x1 = p1(0);
  double y1 = p1(1);
  double x2 = p2(0);
  double y2 = p2(1);

  return std::abs((x2 - x2)*(y1-y0) - (x1-x0)*(y2-y1))/std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

Eigen::Isometry3d updatePoseFromLastResult(const Eigen::Isometry3d& pose, const Eigen::Isometry3d& last_pose, bool update_z, bool update_orientation)
{
  Eigen::Vector3d pose_rpy = rotToNormalizedRpy(pose.linear());

  double roll;
  double pitch;
  if (update_orientation) {
    Eigen::Vector3d previous_rpy = rotToNormalizedRpy(last_pose.linear());
    roll = previous_rpy(0);
    pitch = previous_rpy(1);
  } else {
    roll = pose_rpy(0);
    pitch = pose_rpy(1);
  }
  Eigen::Isometry3d new_init_pose(Eigen::AngleAxisd(pose_rpy(2), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  double z = update_z * last_pose.translation()(2) + (1-update_z) * pose.translation()(2);
  new_init_pose.translation() = Eigen::Vector3d(pose.translation()(0), pose.translation()(1), z);

  return new_init_pose;
}

void pclToVoxbloxCloud(const pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, voxblox::Pointcloud &voxblox_cloud) {
  voxblox_cloud.clear();
  for (const pcl::PointXYZ& p: pcl_cloud) {
    voxblox_cloud.push_back(voxblox::Point(p.x, p.y, p.z));
  }
}

template <> std::string getXmlRpcValueWithDefault(const XmlRpc::XmlRpcValue& dict, const std::string& key, const std::string& default_val) {
  if (dict.hasMember(key) && dict[key].getType() == XmlRpc::XmlRpcValue::TypeString) {
    return static_cast<std::string>(dict[key]);
  } else {
    return default_val;
  }
}

template <> double getXmlRpcValueWithDefault(const XmlRpc::XmlRpcValue& dict, const std::string& key, const double& default_val) {
  if (dict.hasMember(key)) {
    if (dict[key].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      return static_cast<double>(dict[key]);
    } else if (dict[key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      return static_cast<double>(static_cast<int>(dict[key]));
    }
  }
  return default_val;
}

}
