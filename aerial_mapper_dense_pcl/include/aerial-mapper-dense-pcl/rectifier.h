/*
 *    Filename: rectifier.h
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef RECTIFIER_H_
#define RECTIFIER_H_

// NON-SYSTEM
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "aerial-mapper-dense-pcl/common.h"

namespace stereo {

class Rectifier {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Rectifier(const cv::Size& image_resolution)
      : image_resolution_(image_resolution) {
    map_rectify_1_x_.create(image_resolution_.height, image_resolution_.width,
                            CV_32FC1);
    map_rectify_1_y_.create(image_resolution_.height, image_resolution_.width,
                            CV_32FC1);
    map_rectify_2_x_.create(image_resolution_.height, image_resolution_.width,
                            CV_32FC1);
    map_rectify_2_y_.create(image_resolution_.height, image_resolution_.width,
                            CV_32FC1);

    corner_pixel_h_.resize(3, 4);
    corner_pixel_h_.col(0) << 0, 0, 1;
    corner_pixel_h_.col(1) << image_resolution_.width - 1, 0, 1;
    corner_pixel_h_.col(2) << image_resolution_.width - 1,
        image_resolution_.height - 1, 1;
    corner_pixel_h_.col(3) << 0, image_resolution_.height - 1, 1;
  }

  void rectifyStereoPair(const StereoRigParameters& stereo_pair,
                         const cv::Mat& image_left_undistorted,
                         const cv::Mat& image_right_undistorted,
                         RectifiedStereoPair* rectified_stereo_pair);

  cv::Mat computeMask(const Eigen::Matrix3d& T1_rect) const;

  cv::Mat map_rectify_1_x_, map_rectify_1_y_, map_rectify_2_x_,
      map_rectify_2_y_;
  float* map_rectify_1_x_ptr_, *map_rectify_1_y_ptr_, *map_rectify_2_x_ptr_,
      *map_rectify_2_y_ptr_;
  const cv::Size image_resolution_;
  Eigen::MatrixXd corner_pixel_h_;
};

}  // namespace stereo

#endif  // RECTIFIER_H_
