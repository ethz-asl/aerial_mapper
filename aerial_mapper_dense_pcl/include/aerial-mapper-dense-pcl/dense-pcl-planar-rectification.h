/*
 *    Filename: dense-pcl-planar-rectification.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef DENSE_PCL_PLANAR_RECTIFICATION_H_
#define DENSE_PCL_PLANAR_RECTIFICATION_H_

// SYSTEM
#include <memory>

// NON-SYSTEM
#include <aerial-mapper-dense-pcl/Utils.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/pipeline/undistorter-mapped.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <mono_dense_reconstruction_nodes/test/Utils.h>
#include <mono_dense_reconstruction/Optimizer.h>
#include <mono_dense_reconstruction/PlanarRectification.h>
#include <mono_dense_reconstruction/Undistorter.h>
#include <mono_dense_reconstruction/VFQueue.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>


namespace dense_pcl {

struct Settings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  size_t use_every_nth_image = 1;
};

class PlanarRectification {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlanarRectification(const std::shared_ptr<aslam::NCamera> ncameras,
                      const Settings& origin);
  void addFrame(const Pose& T_G_B, const Image& image,
                Aligned<std::vector, Eigen::Vector3d>::type* point_cloud,
                std::vector<int>* point_cloud_intensities = nullptr);
  void addFrames(const Poses& T_G_Bs, const Images& images,
                 Aligned<std::vector, Eigen::Vector3d>::type* point_cloud,
                 std::vector<int>* point_cloud_intensities = nullptr);

 private:
  void showUndistortedCvWindow(const cv::Mat& image_undistorted);
  void publishUndistortedImage(const cv::Mat& image);

  std::unique_ptr<dense::Undistorter> undistorter_;
  static constexpr size_t kFrameIdx = 0u;
  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher pub_undistorted_image_;
  ros::Publisher pub_point_cloud_;
  aslam::Transformation T_G_B_last_;
  cv::Mat image_undistorted_last_;
  bool first_frame_;
  Eigen::Vector3d origin_;
  double downsample_factor_;
  aslam::Transformation T_B_C_;
  Eigen::Matrix3d K_;
  std::unique_ptr<dense::PlanarRectification> planar_rectification_;

  dense::Optimizer::Ptr optimizer_;
  dense::VFQueue::Ptr buffer_;
  std::shared_ptr<aslam::NCamera> ncameras_;
  Settings settings_;

  ros::Publisher pub_ground_points_;
  Eigen::Matrix2Xd border_keypoints_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_pose_C_;
  ros::Publisher pub_pose_C3_;

};

} // namespace dense_pcl

#endif // DENSE_PCL_PLANAR_RECTIFICATION_H_
