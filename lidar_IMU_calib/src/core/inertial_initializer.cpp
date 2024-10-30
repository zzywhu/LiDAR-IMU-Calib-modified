#include <core/inertial_initializer.h>
#include <utils/math_utils.h>

namespace licalib {

bool InertialInitializer::EstimateRotation(
        TrajectoryManager::Ptr traj_manager,
        const Eigen::aligned_vector<LiDAROdometry::OdomData>& odom_data) {

  int flags = kontiki::trajectories::EvalOrientation;
  std::shared_ptr<kontiki::trajectories::SplitTrajectory> p_traj
          = traj_manager->getTrajectory();

  Eigen::aligned_vector<Eigen::Matrix4d> A_vec;
  for (size_t j = 1; j < odom_data.size(); ++j) {
    size_t i = j - 1;
    double ti = odom_data.at(i).timestamp;
    double tj = odom_data.at(j).timestamp;
    if (tj >= p_traj->MaxTime())
      break;
    auto result_i = p_traj->Evaluate(ti, flags);
    auto result_j = p_traj->Evaluate(tj, flags);
    Eigen::Quaterniond delta_qij_imu = result_i->orientation.conjugate()
                                       * result_j->orientation;

    Eigen::Matrix3d R_Si_toS0 = odom_data.at(i).pose.topLeftCorner<3,3>();
    Eigen::Matrix3d R_Sj_toS0 = odom_data.at(j).pose.topLeftCorner<3,3>();
    Eigen::Matrix3d delta_ij_sensor = R_Si_toS0.transpose() * R_Sj_toS0;
    Eigen::Quaterniond delta_qij_sensor(delta_ij_sensor);

    Eigen::AngleAxisd R_vector1(delta_qij_sensor.toRotationMatrix());
    Eigen::AngleAxisd R_vector2(delta_qij_imu.toRotationMatrix());
    double delta_angle = 180 / M_PI * std::fabs(R_vector1.angle() - R_vector2.angle());
    double huber = delta_angle > 1.0 ? 1.0/delta_angle : 1.0;

    Eigen::Matrix4d lq_mat = mathutils::LeftQuatMatrix(delta_qij_sensor);
    Eigen::Matrix4d rq_mat = mathutils::RightQuatMatrix(delta_qij_imu);
    A_vec.push_back(huber * (lq_mat - rq_mat));
  }
  size_t valid_size = A_vec.size();
  if (valid_size < 15) {
    return false;
  }
  Eigen::MatrixXd A(valid_size * 4, 4);
  for (size_t i = 0; i < valid_size; ++i)
    A.block<4, 4>(i * 4, 0) = A_vec.at(i);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Eigen::Quaterniond q_ItoS_est(x);
  Eigen::Vector4d cov = svd.singularValues();
  
  std::cout << "Vector4d cov（2）value ：" << cov(2) << std::endl;
  if (cov(2) > 0.25) {
    q_ItoS_est_ = q_ItoS_est;
    rotaion_initialized_ = true;
    return true;
  } else {
    return false;
  }
}

}
