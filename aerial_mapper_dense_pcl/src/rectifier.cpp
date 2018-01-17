/*
 *    Filename: rectifier.cpp
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-dense-pcl/rectifier.h"

/// The rectification algorithm closely follows:
/// @article{Fusiello:2000:CAR:360401.360413,
///  author = {Fusiello, Andrea and Trucco, Emanuele and Verri, Alessandro},
///  title = {A Compact Algorithm for Rectification of Stereo Pairs},
///  journal = {Mach. Vision Appl.},
///  issue_date = {July 2000},
///  volume = {12},
///  number = {1},
///  month = jul,
///  year = {2000},
///  issn = {0932-8092},
///  pages = {16--22},
///  numpages = {7},
///  url = {http://dx.doi.org/10.1007/s001380050003},
///  doi = {10.1007/s001380050003},
///  acmid = {360413},
///  publisher = {Springer-Verlag New York, Inc.},
///  address = {Secaucus, NJ, USA},
///  keywords = {epipolar geometry, rectification, stereo},
/// }

namespace stereo {

void Rectifier::rectifyStereoPair(const StereoRigParameters& stereo_pair,
                                  const cv::Mat& image_left_undistorted,
                                  const cv::Mat& image_right_undistorted,
                                  RectifiedStereoPair* rectified_stereo_pair) {
  CHECK(rectified_stereo_pair);
  CHECK_EQ(image_resolution_, image_left_undistorted.size());
  CHECK_EQ(image_resolution_, image_right_undistorted.size());

  // New x axis (= direction of the baseline)
  // t_G_C2 - t_G_C1 to ensure that camera 1 is the left camera
  // of the stereo rig.
  const Eigen::Vector3d x = stereo_pair.t_G_C2 - stereo_pair.t_G_C1;
  rectified_stereo_pair->baseline = x.norm();

  // New y axes (orthogonal to new x and old z of camera 1)
  const Eigen::Vector3d y = stereo_pair.R_G_C1.col(2).cross(x);

  // New z axes (orthogonal to baseline and y)
  const Eigen::Vector3d z = x.cross(y);

  // New, rectified rotation matrix of both cameras.
  const Eigen::Matrix3d R_G_C_rect =
      (Eigen::Matrix3d() << x.normalized(), y.normalized(), z.normalized())
          .finished()
          .transpose();
  // Store for later computation of point cloud in world frame.
  rectified_stereo_pair->R_G_C = R_G_C_rect;

  // New projection matrices [Eq. (9)]
  const Eigen::Matrix<double, 3, 4> P1_rect =
      stereo_pair.K *
      (Eigen::Matrix<double, 3, 4>() << R_G_C_rect,
       R_G_C_rect * (-stereo_pair.t_G_C1)).finished();
  const Eigen::Matrix<double, 3, 4> P2_rect =
      stereo_pair.K *
      (Eigen::Matrix<double, 3, 4>() << R_G_C_rect,
       R_G_C_rect * (-stereo_pair.t_G_C2)).finished();

  // Rectifying image transformation.
  const Eigen::Matrix3d Q1 = stereo_pair.K * (stereo_pair.R_G_C1.transpose());
  const Eigen::Matrix3d Q2 = stereo_pair.K * (stereo_pair.R_G_C2.transpose());
  const Eigen::Matrix3d T1_rect = P1_rect.block<3, 3>(0, 0) * Q1.inverse();
  const Eigen::Matrix3d T2_rect = P2_rect.block<3, 3>(0, 0) * Q2.inverse();
  const Eigen::Matrix3f T1_inv = T1_rect.inverse().cast<float>();
  const Eigen::Matrix3f T2_inv = T2_rect.inverse().cast<float>();
  for (int v_rect = 0; v_rect < image_resolution_.height; ++v_rect) {
    map_rectify_1_x_ptr_ = map_rectify_1_x_.ptr<float>(v_rect);
    map_rectify_1_y_ptr_ = map_rectify_1_y_.ptr<float>(v_rect);
    map_rectify_2_x_ptr_ = map_rectify_2_x_.ptr<float>(v_rect);
    map_rectify_2_y_ptr_ = map_rectify_2_y_.ptr<float>(v_rect);
    for (int u_rect = 0; u_rect < image_resolution_.width; ++u_rect) {
      // Apply rectifying image transformation:
      // [u_rect, v_rect, 1.0]^\top = T * [x, y, w]^\top
      // <=> [x, y, w]^\top = T_inv * [u_rect, v_rect, 1.0]^\top
      // u = x / w; v = y / w;

      // Rectify image 1.
      const Eigen::Vector3f xyw_1 =
          T1_inv * (Eigen::Vector3f() << u_rect, v_rect, 1.0).finished();
      CHECK_NE(xyw_1(2), 0.0);
      map_rectify_1_x_ptr_[u_rect] = xyw_1(0) / xyw_1(2);
      map_rectify_1_y_ptr_[u_rect] = xyw_1(1) / xyw_1(2);

      // Rectify image 2.
      const Eigen::Vector3f xyw_2 =
          T2_inv * (Eigen::Vector3f() << u_rect, v_rect, 1.0).finished();
      CHECK_NE(xyw_2(2), 0.0);
      map_rectify_2_x_ptr_[u_rect] = xyw_2(0) / xyw_2(2);
      map_rectify_2_y_ptr_[u_rect] = xyw_2(1) / xyw_2(2);
    }  // for u_rect
  }    // for v_rect

  // Compute the rectified images based on the maps.
  cv::remap(image_left_undistorted, rectified_stereo_pair->image_left,
            map_rectify_1_x_, map_rectify_1_y_, CV_INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  cv::remap(image_right_undistorted, rectified_stereo_pair->image_right,
            map_rectify_2_x_, map_rectify_2_y_, CV_INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  rectified_stereo_pair->mask = computeMask(T1_rect);
}

cv::Mat Rectifier::computeMask(const Eigen::Matrix3d& T1_rect) const {
  cv::Mat mask(image_resolution_.height, image_resolution_.width, CV_8UC1,
               cv::Scalar(0));
  std::vector<std::vector<cv::Point>> corner_pixels_rectified(1);
  for (size_t i = 0u; i < 4u; ++i) {
    const Eigen::Vector3d corner_pixel_rect_h(T1_rect * corner_pixel_h_.col(i));
    corner_pixels_rectified[0].emplace_back(
        corner_pixel_rect_h(0) / corner_pixel_rect_h(2),
        corner_pixel_rect_h(1) / corner_pixel_rect_h(2));
  }
  cv::drawContours(mask, corner_pixels_rectified, 0, cv::Scalar(255), CV_FILLED,
                   8);
  return mask;
}
}  // namespace stereo
