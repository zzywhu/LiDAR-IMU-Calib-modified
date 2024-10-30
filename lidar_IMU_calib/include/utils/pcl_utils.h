#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Eigen>

namespace licalib {

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

typedef pcl::PointXYZRGB colorPointT;
typedef pcl::PointCloud<colorPointT> colorPointCloudT;

struct PointXYZIT {
  PCL_ADD_POINT4D
  uint16_t intensity;
  uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

inline void downsampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
                            float in_leaf_size) {
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(in_cloud);
  sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  sor.filter(*out_cloud);
}

};

POINT_CLOUD_REGISTER_POINT_STRUCT(licalib::PointXYZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint16_t, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (double, timestamp, time))

typedef licalib::PointXYZIT TPoint;
typedef pcl::PointCloud<TPoint> TPointCloud;

inline void TPointCloud2VPointCloud(TPointCloud::Ptr input_pc,
                                    licalib::VPointCloud::Ptr output_pc) {
  output_pc->header = input_pc->header;
  output_pc->height = input_pc->height;
  output_pc->width = input_pc->width;
  output_pc->is_dense = input_pc->is_dense;
  output_pc->resize(output_pc->width * output_pc->height);
  for(int h = 0; h < input_pc->height; h++) {
    for(int w = 0; w < input_pc->width; w++) {
      licalib::VPoint point;
      point.x = input_pc->at(w,h).x;
      point.y = input_pc->at(w,h).y;
      point.z = input_pc->at(w,h).z;
      point.intensity = input_pc->at(w,h).intensity;
      output_pc->at(w,h) = point;
    }
  }
}


#endif
