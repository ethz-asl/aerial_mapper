/*
 *    Filename: ortho-forward-homography.h
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef ORTHO_FORWARD_HOMOGRAPHY_H_
#define ORTHO_FORWARD_HOMOGRAPHY_H_

// SYSTEM
#include <memory>

// NON-SYSTEM
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/pipeline/undistorter-mapped.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <geometry_msgs/PolygonStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>


namespace ortho {

struct Settings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool batch = true;
  double ground_plane_elevation_m = 414.0;
  size_t width_mosaic_pixels = 1000;
  size_t height_mosaic_pixels = 1000;
  Eigen::Vector3d origin{0.0, 0.0, 0.0};
  std::string nframe_id = "map";
  std::string filename_mosaic_output = "/tmp/result.jpg";
};

class OrthoForwardHomography {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OrthoForwardHomography(const std::shared_ptr<aslam::NCamera>& ncameras,
                         const Settings& settings);
  void updateOrthomosaic(const Pose& T_G_B,
                         const Image& image);
  void batch(const Poses& T_G_Bs, const Images& images);

 private:
  void addImage(cv::Mat image_warped, cv::Mat mask_image);
  void addImage(cv::Mat image_warped);
  void prepareBlenderForNextImage();

  void showOrthomosaicCvWindow(cv::Mat current_mosaic);
  void showUndistortedCvWindow(cv::Mat image_undistorted);

  void publishOrthomosaic(cv::Mat current_mosaic);
  void publishUndistortedImage(cv::Mat image);

  cv::Ptr<cv::detail::Blender> blender_;
  std::unique_ptr<aslam::MappedUndistorter> undistorter_;
  Eigen::Matrix2Xd border_keypoints_;
  static constexpr size_t kFrameIdx = 0u;
  std::shared_ptr<aslam::NCamera> ncameras_;
  cv::Mat result_;
  cv::Mat result_mask_;

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher pub_undistorted_image_;
  image_transport::Publisher pub_orthomosaic_image_;

  int image_idx = 0u;

  static constexpr bool ksaveOrthoIncrementallyToFile = false;

  Settings settings_;
};

} // namespace ortho
#endif // ORTHO_FORWARD_HOMOGRAPHY_H_
