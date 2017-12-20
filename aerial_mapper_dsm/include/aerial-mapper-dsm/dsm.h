/*
 *    Filename: dsm.h
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef DSM_H_
#define DSM_H_

// SYSTEM
#include <memory>

// NON-SYSTEM
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

namespace dsm {

struct Settings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int interpolation_radius = 1.0;
  bool adaptive_interpolation = false;
  double center_easting = 0.0;
  double center_northing = 0.0;
  bool use_multi_threads = true;
};

class Dsm {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Dsm(const Settings& settings, grid_map::GridMap* map);

  void process(
      const AlignedType<std::vector, Eigen::Vector3d>::type& point_cloud,
      grid_map::GridMap* map);

 private:
  void initializeAndFillKdTree(
      const AlignedType<std::vector, Eigen::Vector3d>::type& point_cloud);

  void updateElevationLayer(grid_map::GridMap* map);

  void updateElevationLayerMultiThreaded(grid_map::GridMap* map);

  void printParams();

  Settings settings_;

  // kd Tree
  static constexpr size_t kMaxLeaf = 10u;
  static constexpr size_t kDimensionKdTree = 2u;
  typedef PointCloudAdaptor<PointCloud<double> > PC2KD;
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Adaptor<double, PC2KD>, PC2KD, kDimensionKdTree>
      my_kd_tree_t;
  PointCloud<double> cloud_kdtree_;
  std::unique_ptr<my_kd_tree_t> kd_tree_;
  std::unique_ptr<PC2KD> pc2kd_;

  // Multi-threading.
  std::unordered_map<size_t, grid_map::Index> map_sample_to_cell_index_;
  std::vector<size_t> samples_idx_range_;
};

}  // namespace dsm

#endif  // DSM_H_
