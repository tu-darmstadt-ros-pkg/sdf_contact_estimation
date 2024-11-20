#ifndef SDF_CONTACT_ESTIMATION_UTILS_H
#define SDF_CONTACT_ESTIMATION_UTILS_H

#include <Eigen/Eigen>
#include <hector_math/types/eigen.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <voxblox/core/common.h>

namespace sdf_contact_estimation {

template<typename T>
using Vector3T = Eigen::Matrix<T, 3, 1>;

template<typename T>
using Affine3T = Eigen::Transform<T, 3, Eigen::Affine>;

Eigen::Quaterniond rpyToRot(double roll, double pitch, double yaw);
Eigen::Vector3d rotToNormalizedRpy(const Eigen::Matrix3d &rot);

double distanceToLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& x);
double distanceToLine2D(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& x);

bool isOnLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& x);

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);

/// Computes a rotation from vector v1 to vector v2.
/// The remaining free parameter (rotation around the vector) is determined by aligning the z-axis
/// \param v1
/// \param v2
/// \return
Eigen::Matrix3d computeGravityAlignedRotationFromTo(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

template <typename T> std::vector<T> computeForceAngleStabilities(const std::vector<Vector3T<T>>& contact_points, const Vector3T<T>& external_force, const Vector3T<T>& com) {
  std::vector<T> stabilities;
  for (unsigned int i = 0; i < contact_points.size(); i++) {
    Vector3T<T> contact_point_p1 = contact_points[(i+1) % contact_points.size()];

    Vector3T<T> axis = contact_point_p1 - contact_points[i]; // a_i
    Eigen::Matrix<T, 3, 3> identity = Eigen::Matrix<T, 3, 3>::Identity(); // I
    Vector3T<T> axis_norm = axis / axis.norm(); // \hat{a}_i
    Eigen::Matrix<T, 3, 3> p1 = identity - axis_norm * axis_norm.transpose(); // I - \hat{a}_i * \hat{a}_i^T
    Vector3T<T> axis_normal = p1 * (contact_point_p1 - com); // l_i
    Vector3T<T> axis_normal_norm = axis_normal / axis_normal.norm();
    Vector3T<T> force_component = p1 * external_force; // f_i -> can be precomputed
    Vector3T<T> force_component_norm = force_component / force_component.norm(); // \hat{f}_i
    Vector3T<T> distance = -axis_normal + (axis_normal.dot(force_component_norm)) * force_component_norm; // d_i


    T t1 = force_component_norm.cross(axis_normal_norm).dot(axis_norm);
    T t2 = axis_normal_norm.dot(force_component_norm);
    T signed_angle = atan2(t1, t2);
//      T angle = acos(axis_normal_norm.dot(force_component_norm));
    T stability = signed_angle * distance.norm() * force_component.norm();
    stabilities.push_back(stability);
  }

  return stabilities;
}

template <typename T> std::vector<T> computeForceAngleStabilitiesWithGravity(const std::vector<Vector3T<T>>& contact_points, const Vector3T<T>& com) {
  return computeForceAngleStabilities(contact_points, Vector3T<T>(T(0), T(0), T(-9.81)), com);
}

template<typename T>
inline bool equals2d(const T& p1, const T& p2) {
  double e = 1e-8;
  return (std::abs(p1.x - p2.x) < e) && (std::abs(p1.y - p2.y) < e);
}

Eigen::Isometry3d updatePoseFromLastResult(const Eigen::Isometry3d& pose, const Eigen::Isometry3d& last_pose, bool update_z=true, bool update_orientation=true);

void pclToVoxbloxCloud(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, voxblox::Pointcloud& voxblox_cloud);

template <typename T> std::string vectorToString(const std::vector<T>& v) {
  std::stringstream ss;
  ss << "[";
  for (const T& el: v) {
    ss << el << ", ";
  }
  ss << "]";
  return ss.str();
}

template <typename T> T getXmlRpcValueWithDefault(const XmlRpc::XmlRpcValue& dict, const std::string& key, const T& default_val);

template <typename T> std::vector<T> getXmlRpcValueWithDefault(const XmlRpc::XmlRpcValue& dict, const std::string& key, const std::vector<T>& default_val) {
  if (dict.hasMember(key) && dict[key].getType() == XmlRpc::XmlRpcValue::TypeArray) {
    const XmlRpc::XmlRpcValue& array = dict[key];
    std::vector<T> vec;
    vec.reserve(array.size());
    for (int j; j < array.size(); ++j) {
      vec.push_back(array[j]);
    }
    return vec;
  } else {
    return default_val;
  }
}

inline int mod(int k, int n) {
  return ((k %= n) < 0) ? k+n : k;
}

template <typename Scalar>
hector_math::Vector3List<Scalar> supportPolygonAngleFilter(const hector_math::Vector3List<Scalar>& convex_hull_points, double angle_rad) {
  if (convex_hull_points.size() < 3 || angle_rad <= 0.0) {
    return convex_hull_points;
  }
  hector_math::Vector3List<Scalar> points_filtered;
  points_filtered.reserve(convex_hull_points.size());
  double boundary = std::abs(std::cos(M_PI - angle_rad));
  for (int i = 0; i < convex_hull_points.size(); ++i) {
    // find neighboring points
    size_t pm1 = mod(i - 1, convex_hull_points.size());
    size_t pp1 = mod(i + 1, convex_hull_points.size());

    // Vectors that meet at this corner, in 2D
    Eigen::Vector3d v1 = convex_hull_points[pp1] - convex_hull_points[i];
    v1.z() = 0;
    Eigen::Vector3d v2 = convex_hull_points[i] - convex_hull_points[pm1];
    v2.z() = 0;

    // Angle between vectors
    double cos_alpha = std::abs(v1.dot(v2)/(v1.norm() * v2.norm()));
    bool keep_point = cos_alpha <= boundary + 0.001;
    if (keep_point) {
      points_filtered.push_back(convex_hull_points[i]);
    }
//    else {
//      ROS_INFO_STREAM("size: " << convex_hull_points.size());
//      ROS_INFO_STREAM("pm1 " << pm1 << convex_hull_points[pm1]);
//      ROS_INFO_STREAM("i " << i << convex_hull_points[i]);
//      ROS_INFO_STREAM("pp1 " << pp1 << convex_hull_points[pp1]);
//      ROS_INFO_STREAM("v1 " << v1);
//      ROS_INFO_STREAM("v2 " << v2);
//      ROS_INFO_STREAM("Point " << i << ": cos alpha: " << cos_alpha << "boundary: " << boundary << ", keep: " << keep_point);
//      ROS_INFO_STREAM("--------------");
//    }
  }
  return points_filtered;
}

}

#endif
