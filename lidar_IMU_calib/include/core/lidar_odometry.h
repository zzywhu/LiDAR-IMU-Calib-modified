#ifndef CALIBR_LIDAR_ODOMETRY_H
#define CALIBR_LIDAR_ODOMETRY_H

#include <utils/vlp_common.h>
#include <utils/eigen_utils.hpp>
#include <utils/pcl_utils.h>
#include <pclomp/ndt_omp.h>

namespace licalib {

class LiDAROdometry {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<LiDAROdometry> Ptr;

  struct OdomData {
    double timestamp;
    Eigen::Matrix4d pose; // cur scan to first scan
  };

  explicit LiDAROdometry(double ndtResolution = 0.5);

  static pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr ndtInit(
          double ndt_resolution);

  void feedScan(double timestamp,
                VPointCloud::Ptr cur_scan,
                Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity(),
                const bool update_map = true);

  void clearOdomData();

  void setTargetMap(VPointCloud::Ptr map_cloud_in);

  void saveTargetMap(const std::string& path) const {
    std::cout << "Save NDT target map to " << path
              << "; size: " << map_cloud_->size() << std::endl;
    pcl::io::savePCDFileASCII(path, *map_cloud_);
  }

  const VPointCloud::Ptr getTargetMap(){
    return map_cloud_;
  }

  const pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr& getNDTPtr() const {
    return ndt_omp_;
  }

  const Eigen::aligned_vector<OdomData> &get_odom_data() const {
    return odom_data_;
  }

private:

  void registration(const VPointCloud::Ptr& cur_scan,
                    const Eigen::Matrix4d& pose_predict,
                    Eigen::Matrix4d& pose_out,
                    VPointCloud::Ptr scan_in_target);

  void updateKeyScan(const VPointCloud::Ptr& cur_scan, const OdomData& odom_data);

  bool checkKeyScan(const OdomData& odomdata);

  // Normalize angle to be between [-180, 180]
  static inline double normalize_angle(double ang_degree) {
    if(ang_degree > 180)
      ang_degree -= 360;

    if(ang_degree < -180)
      ang_degree += 360;
    return ang_degree;
  }

private:

  VPointCloud::Ptr map_cloud_;

  pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr ndt_omp_;

  std::vector<size_t> key_frame_index_;
  Eigen::aligned_vector<OdomData> odom_data_;
};


}


#endif // CALIBR_LIDAR_ODOMETRY_H
