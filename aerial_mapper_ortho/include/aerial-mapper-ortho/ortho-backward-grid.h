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
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <geometry_msgs/PolygonStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

namespace ortho {

enum Mode { Incremental, Batch };

struct SettingsGrid {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double orthomosaic_easting_min = 0.0;
  double orthomosaic_northing_min = 0.0;
  double orthomosaic_easting_max = 0.0;
  double orthomosaic_northing_max = 0.0;
  double orthomosaic_resolution = 1.0;
  bool show_orthomosaic_opencv = true;
  bool save_orthomosaic_jpg = true;
  std::string orthomosaic_jpg_filename = "";
  double orthomosaic_elevation_m = 0.0;
  bool use_digital_elevation_map = false;
  Mode mode = Mode::Batch;
};

class OrthoBackwardGrid {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OrthoBackwardGrid(const std::shared_ptr<aslam::NCamera> ncameras,
                    const Poses& T_G_Bs, const Images& images,
                    const SettingsGrid& settings);
  void processBatch(const Poses& T_G_Cs,
                    const Images& images);
  void processIncremental(const Poses& T_G_Cs,
                          const Images& images);

 private:
  void printParams() const;
  std::shared_ptr<aslam::NCamera> ncameras_;
  static constexpr size_t kFrameIdx = 0u;
  SettingsGrid settings_;
};
} // namespace ortho

#endif  // ORTHO_BACKWARD_GRID_H_
