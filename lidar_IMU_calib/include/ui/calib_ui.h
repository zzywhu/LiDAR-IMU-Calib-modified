#ifndef CALIB_UI_H
#define CALIB_UI_H

#include <ui/calib_helper.h>
#include <pangolin/pangolin.h>

using namespace licalib;

struct TranslationVector {
  Eigen::Vector3d trans = Eigen::Vector3d(0,0,0);
};

class CalibInterface : public CalibrHelper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibInterface(ros::NodeHandle& nh);

  void initGui();

  void renderingLoop();

  void showCalibResult();

  void resetModelView() {
    s_cam_.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0,40,0,0,0,pangolin::AxisNegY));
  }

private:
  static constexpr int UI_WIDTH = 300;

  pangolin::View *pointcloud_view_display_;
  pangolin::OpenGlRenderState s_cam_;

  std::vector<pangolin::Colour> pangolin_colors_;

  pangolin::Var<std::string> A_string;
  pangolin::Var<bool> show_surfel_map_;
  pangolin::Var<bool> show_all_association_points_;
  pangolin::Var<bool> optimize_time_offset_;
  pangolin::Var<int>  show_lidar_frame_;

  pangolin::Var<TranslationVector> show_p_IinL_;
  pangolin::Var<TranslationVector> show_q_ItoL_;
  //pangolin::Var<TranslationVector> show_gravity_;
  //pangolin::Var<TranslationVector> show_gyro_bias_;
  //pangolin::Var<TranslationVector> show_acce_bias_;
  //pangolin::Var<double> show_time_offset_;
};


#endif
