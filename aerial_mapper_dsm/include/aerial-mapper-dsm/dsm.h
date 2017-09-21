/*
 *    Filename: dsm.h
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef DSM_H_
#define DSM_H_

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
#include <gdal/cpl_string.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>
#include <image_transport/image_transport.h>
#include <maplab-common/progress-bar.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>


namespace dsm {

struct Settings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int color_palette = 6;
  int resolution = 6;
  bool show_output = true;
  int interpolation_radius = 1.0;
  bool adaptive_interpolation = false;
  Eigen::Vector3d origin = Eigen::Vector3d(0.0, 0.0, 0.0);
};

class Dsm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Dsm(const Settings& settings);

  void process(
      const Aligned<std::vector, Eigen::Vector3d>::type& pointcloud);

 private:
  void printParams();
  Settings settings_;
};

} // namespace dsm

#endif // DSM_H_
