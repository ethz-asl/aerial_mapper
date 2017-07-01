/*
 *    Filename: ortho-from-pcl.h
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef ORTHO_FROM_PCL_H_
#define ORTHO_FROM_PCL_H_

// SYSTEM
#include <fstream>
#include <iomanip>
#include <memory>

// NON-SYSTEM
#include <aerial-mapper-utils/utils-color-palette.h>
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/pipeline/undistorter-mapped.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <image_transport/image_transport.h>
#include <maplab-common/progress-bar.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace ortho {

struct Settings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int color_palette = 6;
  int resolution = 2;
  bool show_output = true;
  int interpolation_radius = 10;
  bool adaptive_interpolation = false;
};

class OrthoFromPcl {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OrthoFromPcl(const Aligned<std::vector,
               Eigen::Vector3d>::type& pointcloud,
               const std::vector<int>& intensities,
               const Settings& settings);

  void process(const Aligned<std::vector,
               Eigen::Vector3d>::type& pointcloud,
               const std::vector<int>& intensities);
 private:
  Settings settings_;
  PointCloud<double> cloud_kdtree_;
};
}  // namespace ortho
#endif  // ORTHO_FROM_PCL_H_
