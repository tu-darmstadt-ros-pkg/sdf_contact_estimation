#ifndef SDF_CONTACT_ESTIMATION_SDF_MODEL_H
#define SDF_CONTACT_ESTIMATION_SDF_MODEL_H

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh_layer.h>

#include <sdf_contact_estimation/sdf/interpolated_voxblox_tsdf.h>
#include <sdf_contact_estimation/sdf/interpolated_voxblox_esdf.h>

typedef cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxTSDF InterpolatedTsdf;
typedef cartographer::mapping_3d::scan_matching::InterpolatedVoxbloxESDF InterpolatedEsdf;

enum SdfType {
  NONE,
  TSDF,
  ESDF
};

namespace sdf_contact_estimation {

class SdfModel {
public:
  SdfModel();
  explicit SdfModel(const ros::NodeHandle& nh);
  bool loadFromServer(const ros::NodeHandle& nh);

  /// Access SDF
  template<typename T> T getSdf(const T& x, const T& y, const T& z) const {
    switch (sdf_type_) {
      case NONE:
        ROS_ERROR_STREAM("SdfModel has not been initialized yet.");
        return T(0.0);
      case TSDF:
        if (tsdf_) {
          return tsdf_->GetSDF<T>(x, y, z, 1);
        } else {
          ROS_ERROR_STREAM("TSDF is null.");
          return T(0.0);
        }
      case ESDF:
        if (esdf_) {
          return esdf_->GetSDF<T>(x, y, z, 1);
        } else {
          ROS_ERROR("ESDF is null.");
          return T(0.0);
        }
      default:
        ROS_ERROR_STREAM("Unkown SDF type.");
        return T(0.0);
    }
  }
  template <typename T>
  T getDistanceAndGradient(const Eigen::Matrix<T, 3, 1>& position, Eigen::Matrix<T, 3, 1>& gradient) const {
    switch (sdf_type_) {
      case NONE:
        ROS_ERROR_STREAM("SdfModel has not been initialized yet.");
        return T(0.0);
      case TSDF:
        if (tsdf_) {
          ROS_ERROR_STREAM("Gradients not supported for TSDF");
          return T(0.0);
        } else {
          ROS_ERROR_STREAM("TSDF is null.");
          return T(0.0);
        }
      case ESDF:
        if (esdf_) {
          return esdf_->GetSDFAndGradient<double>(position.x(), position.y(), position.z(), gradient, 1);
        } else {
          ROS_ERROR("ESDF is null.");
          return T(0.0);
        }
      default:
        ROS_ERROR_STREAM("Unkown SDF type.");
        return T(0.0);
    }
  }

  /// Set SDF
  void loadCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, float truncation_distance, float voxel_size,
                 bool use_esdf = false);
  bool loadSdfFromFile(std::string file_path, float truncation_distance, bool publish_mesh = true);

  void loadTsdf(const std::shared_ptr<voxblox::TsdfMap>& tsdf, float truncation_distance, bool compute_esdf = false,
                bool publish_mesh = true);
  bool loadTsdfFromFile(const std::string& tsdf_file_path, float truncation_distance, bool compute_esdf = false,
                        bool publish_mesh = true);


  void loadEsdf(const std::shared_ptr<voxblox::EsdfMap>& esdf, float truncation_distance, bool publish_mesh = true);
  bool loadEsdfFromFile(const std::string& esdf_file_path, float max_truncation_distance, bool publish_mesh = true);

  void setWorldFrame(const std::string& world_frame);
  std::string getWorldFrame() const;


  /// Get SDF
  std::shared_ptr<InterpolatedTsdf> getTsdf() const;
  std::shared_ptr<InterpolatedEsdf> getEsdf() const;
  bool saveTsdfToFile(const std::string& file_path);
  bool saveEsdfToFile(const std::string& file_path);

  void publishSdfMesh() const;

  SdfType getSdfType() const;
  bool isLoaded() const;

  bool isEmpty() const;
private:
  void setTsdfMap(const std::shared_ptr<voxblox::TsdfMap>& tsdf, float truncation_distance);
  void setEsdfMap(const std::shared_ptr<voxblox::EsdfMap>& esdf, float truncation_distance);

  static std::shared_ptr<voxblox::TsdfMap> computeTsdf(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                       float truncation_distance, float voxel_size);
  static std::shared_ptr<voxblox::EsdfMap> computeEsdf(const std::shared_ptr<voxblox::TsdfMap>& tsdf);
  static std::shared_ptr<voxblox::MeshLayer> computeMesh(const std::shared_ptr<voxblox::TsdfMap>& tsdf);
  static std::shared_ptr<voxblox::MeshLayer> computeMesh(const std::shared_ptr<voxblox::EsdfMap>& esdf);

  SdfType sdf_type_;
  std::shared_ptr<InterpolatedTsdf> tsdf_;
  std::shared_ptr<InterpolatedEsdf> esdf_;

  bool publishing_;
  std::string world_frame_;
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::Publisher mesh_pub_;
  ros::Publisher tsdf_slice_pub_;
  ros::Publisher esdf_slice_pub_;
};

typedef std::shared_ptr<SdfModel> SdfModelPtr;
typedef std::shared_ptr<const SdfModel> SdfModelConstPtr;

}  // namespace sdf_contact_estimation

#endif
