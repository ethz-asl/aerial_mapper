/*
 *    Filename: ortho-from-pcl.h
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef ORTHO_FROM_PCL_H_
#define ORTHO_FROM_PCL_H_

// SYSTEM
#include <iomanip>
#include <memory>
#include <string>

// NON-SYSTEM
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <aslam/cameras/ncamera.h>
#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

namespace ortho {

struct Settings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool show_orthomosaic_opencv = false;
  int interpolation_radius = 2;
  bool use_adaptive_interpolation = false;
  bool save_orthomosaic_jpg = false;
  std::string orthomosaic_jpg_filename = "";
};

class OrthoFromPcl {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OrthoFromPcl(const Settings& settings);

  void process(const AlignedType<std::vector,
               Eigen::Vector3d>::type& pointcloud,
               const std::vector<int>& intensities,
               grid_map::GridMap* map) const;

 private:
  void printParams() const;
  Settings settings_;
  PointCloud<double> cloud_kdtree_;
};
}  // namespace ortho
#endif  // ORTHO_FROM_PCL_H_
