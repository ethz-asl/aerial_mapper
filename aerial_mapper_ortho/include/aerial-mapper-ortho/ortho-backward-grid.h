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

// NON-SYSTEM
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

class OrthoBackwardGrid {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OrthoBackwardGrid();
  OrthoBackwardGrid(const std::shared_ptr<aslam::NCamera> ncameras,
                    const Poses& T_G_Bs, const Images& images);
  void processBatch(const Poses& T_G_Cs,
                    const Images& images);
  void processIncremental(const Poses& T_G_Cs,
                          const Images& images);

 private:
  bool withinImageBoxWithBorder(const Eigen::Vector2d& keypoint,
                                const aslam::Camera& camera);

  std::shared_ptr<aslam::NCamera> ncameras_;
  static constexpr size_t kFrameIdx = 0u;
};
}

#endif  // ORTHO_BACKWARD_GRID_H_
