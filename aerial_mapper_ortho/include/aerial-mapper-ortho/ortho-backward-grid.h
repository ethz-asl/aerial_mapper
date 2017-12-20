/*
 *    Filename: ortho-backward-grid.h
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef ORTHO_BACKWARD_GRID_H_
#define ORTHO_BACKWARD_GRID_H_

// SYSTEM
#include <memory>
#include <string>

// NON-SYSTEM
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/pipeline/undistorter-mapped.h>
#include <aslam/pipeline/visual-npipeline.h>
#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

namespace ortho {

struct Settings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool show_orthomosaic_opencv = true;
  bool save_orthomosaic_jpg = true;
  std::string orthomosaic_jpg_filename = "";
  double orthomosaic_elevation_m = 0.0;
  bool use_digital_elevation_map = true;
  bool colored_ortho = false;
  bool use_multi_threads = true;
};

class OrthoBackwardGrid {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OrthoBackwardGrid(const std::shared_ptr<aslam::NCamera> ncameras,
                    const Settings& settings, grid_map::GridMap* map = nullptr);

  void process(const Poses& T_G_Bs, const Images& images,
               grid_map::GridMap* map) const;

 private:

  void updateOrthomosaicLayer(const Poses& T_G_Cs, const Images& images,
                              grid_map::GridMap* map) const;

  void updateOrthomosaicLayerMultiThreaded(const Poses& T_G_Cs,
                                           const Images& images,
                                           grid_map::GridMap* map) const;

  void printParams() const;

  std::shared_ptr<aslam::NCamera> ncameras_;
  static constexpr size_t kFrameIdx = 0u;
  Settings settings_;

  // Multi-threading.
  std::unordered_map<size_t, grid_map::Index> map_sample_to_cell_index_;
  std::vector<size_t> samples_idx_range_;
};
}  // namespace ortho

#endif  // ORTHO_BACKWARD_GRID_H_
