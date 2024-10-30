#ifndef CALIB_HELPER_H
#define CALIB_HELPER_H

#include <core/inertial_initializer.h>
#include <core/lidar_odometry.h>
#include <core/surfel_association.h>
#include <utils/dataset_reader.h>
#include <core/scan_undistortion.h>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

namespace licalib {

class CalibrHelper {
public:

  enum CalibStep {
    Error = 0,
    Start,
    InitializationDone,
    DataAssociationDone,
    BatchOptimizationDone,
    RefineDone
  };

  explicit CalibrHelper(ros::NodeHandle& nh);

  bool createCacheFolder(const std::string& bag_path);
  
  void Initialization();

  void DataAssociation();

  void BatchOptimization();

  void Refinement();

  void saveCalibResult(const std::string& calib_result_file) const;

  void saveMap() const;

protected:

  void Mapping(bool relocalization = false);

  /// dataset
  std::string cache_path_;
  std::string topic_imu_;
  std::string bag_path_;

  /// optimization
  CalibStep calib_step_;
  int iteration_step_;
  bool opt_time_offset_;

  /// lidar odometry
  double map_time_;
  double ndt_resolution_;
  double scan4map_time_;

  /// data association
  double associated_radius_;
  double plane_lambda_;

  std::shared_ptr<IO::LioDataset> dataset_reader_;
  InertialInitializer::Ptr rotation_initializer_;
  TrajectoryManager::Ptr traj_manager_;
  LiDAROdometry::Ptr lidar_odom_;
  SurfelAssociation::Ptr surfel_association_;

  ScanUndistortion::Ptr scan_undistortion_;
};

}

#endif
