/*
 * fw_online_digital_elevation_map_node.hpp
 *
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef FW_ONLINE_DIGITAL_ELEVATION_MAP_H_
#define FW_ONLINE_DIGITAL_ELEVATION_MAP_H_

#include <fstream>
#include <iomanip>
#include <memory>

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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <multiagent-mapping-common/progress-bar.h>

#include "fw_online_digital_elevation_map_node/utils-color-palette.h"
#include "fw_online_digital_elevation_map_node/utils-nearest-neighbor.h"


class FwOnlineDigitalElevationMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor to load pointcloud from ros message.
  FwOnlineDigitalElevationMap(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                              const Eigen::Vector3d& origin);

  /// Load pointcloud from ros message.
  void loadPointcloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                      const Eigen::Vector3d& origin,
                      Aligned<std::vector, Eigen::Vector3d>::type* pointcloud);

  /// Load pointcloud from file.
  void loadPointcloud(const std::string& filename,
                      Aligned<std::vector, Eigen::Vector3d>::type* pointcloud);

  void writeDataToDEMGeoTiff(const cv::Mat& ortho_image, const Eigen::Vector2d& xy,
                             const std::string filename);

  void writeDataToDEMGeoTiffColor(const cv::Mat& ortho_image, const Eigen::Vector2d& xy,
                                  const std::string filename);

  void process(const Aligned<std::vector, Eigen::Vector3d>::type& pointcloud);
};

#endif




