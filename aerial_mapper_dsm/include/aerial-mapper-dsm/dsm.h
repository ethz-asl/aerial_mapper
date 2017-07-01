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


//DECLARE_string(DEM_filename_xyz);
//DECLARE_string(DEM_output_folder);
//DECLARE_double(DEM_interpolation_radius);
//DECLARE_int32(DEM_color_palette);
//DECLARE_double(DEM_resolution);
//DECLARE_int32(DEM_UTM_code);
//DECLARE_bool(DEM_show_output);
//DECLARE_bool(DEM_save_cv_mat_height_map);
//DECLARE_double(DEM_easting_min);
//DECLARE_double(DEM_northing_min);
//DECLARE_double(DEM_easting_max);
//DECLARE_double(DEM_northing_max);


namespace dsm {

struct Settings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int color_palette = 6;
  int resolution = 2;
  bool show_output = true;
  int interpolation_radius = 10;
  bool adaptive_interpolation = false;
  Eigen::Vector3d origin = Eigen::Vector3d(0.0, 0.0, 0.0);
};

class Dsm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor to load pointcloud from ros message.
  //Dsm(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
  //    const Eigen::Vector3d& origin);
  Dsm(const Settings& settings);

  void process(
      const Aligned<std::vector, Eigen::Vector3d>::type& pointcloud);

 private:
  void printParams();
  Settings settings_;
};

} // namespace dsm

#endif // DSM_H_
