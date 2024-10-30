#ifndef CALIBR_INERTIALINITIALIZER_H
#define CALIBR_INERTIALINITIALIZER_H

#include <ros/ros.h>
#include <utils/eigen_utils.hpp>
#include <core/trajectory_manager.h>
#include <core/lidar_odometry.h>

namespace licalib {


class InertialInitializer {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<InertialInitializer> Ptr;

  explicit InertialInitializer() : rotaion_initialized_(false),
                                   q_ItoS_est_(Eigen::Quaterniond::Identity()) {
  }

  bool EstimateRotation(TrajectoryManager::Ptr traj_manager,
                        const Eigen::aligned_vector<LiDAROdometry::OdomData>& odom_data);

  bool isInitialized() {
    return rotaion_initialized_;
  }

  Eigen::Quaterniond getQ_ItoS() {
    return q_ItoS_est_;
  }


private:
  bool rotaion_initialized_;
  Eigen::Quaterniond q_ItoS_est_;

};


}

#endif //CALIBR_INERTIALINITIALIZER_H
