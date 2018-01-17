/*
 *    Filename: common.h
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/pipeline/undistorter-mapped.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <minkindr_conversions/kindr_msg.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <aerial-mapper-utils/utils-nearest-neighbor.h>

namespace stereo {

struct Settings {
  size_t use_every_nth_image = 1;
  bool images_need_undistortion = false;
  bool show_rectification = true;
};

struct StereoRigParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  size_t use_every_nth_image = 1;
  Eigen::Matrix3d K;

  /// Resolution of the images. Left/right assumed to be identical.
  cv::Size image_size;

  /// Position of camera 1 (left) in global/world frame.
  Eigen::Vector3d t_G_C1;

  /// Position of camera 2 (right) in global/world frame.
  Eigen::Vector3d t_G_C2;

  /// Orientation of camera 1 (left) in global/world frame.
  Eigen::Matrix3d R_G_C1;

  /// Orientation of camera 2 (right) in global/world frame.
  Eigen::Matrix3d R_G_C2;

  /// Rectifiying image transformation for camera 1 (left).
  Eigen::Matrix4d T_1;

  /// Rectifying image transformation for camera 2 (right).
  Eigen::Matrix4d T_2;
};

struct RectifiedStereoPair {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double baseline;
  Eigen::Matrix3d R_G_C;
  cv::Mat mask;
  cv::Mat image_left;
  cv::Mat image_right;
};

struct DensifiedStereoPair {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  cv::Mat disparity_map;
  cv::Mat_<cv::Vec3f> point_cloud;
  AlignedType<std::vector, Eigen::Vector3d>::type point_cloud_eigen;
  std::vector<int> point_cloud_intensities;
};

struct BlockMatchingParameters {
  // Uses SGBM if "use_BM" is false.
  bool use_BM = false;

  struct SGBM {
    int min_disparity = 1;  // 36;
    int num_disparities = 80;  // 128;
    int pre_filter_cap = 35;  // 1;
    int uniqueness_ratio = 10;  // 64;
    int speckle_window_size = 100;  // 0;
    int speckle_range = 20;  // 0;
    int disp_12_max_diff = 0;
    int p1 = 120;  // 350;
    int p2 = 250;  // 870;
    int block_size = 9;  // 5;
  } sgbm;

  struct BM {
    int min_disparity = 1;
    int num_disparities = 80;
    int pre_filter_cap = 31;
    int pre_filter_size = 9;
    int uniqueness_ratio = 80;
    int texture_threshold = 20;
    int speckle_window_size = 100;
    int speckle_range = 5;
    int disp_12_max_diff = 0;
    int block_size = 15;
  } bm;
};

typedef kindr::minimal::QuatTransformation Pose;
typedef std::vector<Pose> Poses;
typedef cv::Mat Image;
typedef std::vector<Image> Images;

}  // namespace stereo

#endif  // COMMON_H_
