Prerequisites
ROS(Melodic) Ceres(1.14.0) pcl(1.8.1)

Install
init ROS workspace
cd ~/lidar-imu_ws/src catkin_init_workspace

ndt_omp
wstool init wstool merge lidar_IMU_calib/depend_pack.rosinstall wstool update

Pangolin
cd lidar_IMU_calib ./build_submodules.sh

build
cd ../.. catkin_make source ./devel/setup.bash

Run the calibration:
roslaunch li_calib my_calib.launch
