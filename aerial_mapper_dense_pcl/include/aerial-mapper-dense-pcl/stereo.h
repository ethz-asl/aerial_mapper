/*
 *    Filename: stereo.h
 *  Created on: Nov 01, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef STEREO_H_
#define STEREO_H_

// SYSTEM
#include <algorithm>
#include <memory>

// NON-SYSTEM
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
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// PACKAGE
#include "aerial-mapper-dense-pcl/densifier.h"
#include "aerial-mapper-dense-pcl/rectifier.h"
#include "aerial-mapper-dense-pcl/common.h"

namespace stereo {

class Stereo {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Stereo(const std::shared_ptr<aslam::NCamera> ncameras,
         const Settings& settings,
         const BlockMatchingParameters& block_matching_params);

  void addFrames(const Poses& T_G_Bs, const Images& images,
                 AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud,
                 std::vector<int>* point_cloud_intensities = nullptr);

  void addFrame(const Pose& T_G_B, const Image& image,
                AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud,
                std::vector<int>* point_cloud_intensities = nullptr);

  void processStereoFrame(
      AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud,
      std::vector<int>* point_cloud_intensities);

  void undistortRawImages(const cv::Mat& image_distorted_1,
                          const cv::Mat& image_distorted_2,
                          cv::Mat* image_undistorted_1,
                          cv::Mat* image_undistorted_2) const;

  void visualizeRectification(
      const cv::Mat& image_undistorted_1, const cv::Mat& image_undistorted_2,
      const cv::Mat& image_undistorted_rectified_1,
      const cv::Mat& image_undistorted_rectified_2) const;

  static constexpr size_t kCameraIdxLeft = 0u;
  static constexpr size_t kCameraIdxRight = 1u;
  static constexpr size_t kFrameIdx = 0u;

  /// ROS
  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher pub_undistorted_image_;
  ros::Publisher pub_point_cloud_;
  sensor_msgs::PointCloud2 point_cloud_ros_msg_;

  std::unique_ptr<Rectifier> rectifier_;
  std::unique_ptr<Densifier> densifier_;
  std::unique_ptr<aslam::MappedUndistorter> undistorter_;

  bool first_frame_;

  std::shared_ptr<aslam::NCamera> ncameras_;
  StereoRigParameters stereo_rig_params_;
  Settings settings_;
  aslam::Transformation T_B_C_;
  cv::Mat image_distorted_1_;
  cv::Mat image_distorted_2_;
};

}  // namespace stereo

#endif  // STEREO_H_
