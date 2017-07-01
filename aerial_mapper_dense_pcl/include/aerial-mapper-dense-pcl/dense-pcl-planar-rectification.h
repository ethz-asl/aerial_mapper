#ifndef FW_ONLINE_PLANAR_RECTIFICATION_H_
#define FW_ONLINE_PLANAR_RECTIFICATION_H_

#include <memory>

#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/pipeline/undistorter-mapped.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <aerial-mapper-dense-pcl/Utils.h>
#include <mono_dense_reconstruction_nodes/test/Utils.h>
#include <mono_dense_reconstruction/PlanarRectification.h>
#include <mono_dense_reconstruction/Undistorter.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <mono_dense_reconstruction/Optimizer.h>
#include <mono_dense_reconstruction/VFQueue.h>

class FwOnlinePlanarRectification {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FwOnlinePlanarRectification(std::string ncameras_yaml_path_filename,
                              const Eigen::Vector3d& origin);
  void addFrame(const aslam::Transformation& T_G_B, const cv::Mat& image);

 private:
  void loadCameraRig(std::string ncameras_yaml_path_filename);
  void showUndistortedCvWindow(const cv::Mat& image_undistorted);
  void publishUndistortedImage(const cv::Mat& image);

  std::unique_ptr<dense::Undistorter> undistorter_;
  static constexpr size_t kFrameIdx = 0u;
  std::shared_ptr<aslam::NCamera> ncameras_;
  ros::NodeHandle node_handle_;
  std::unique_ptr<image_transport::ImageTransport> image_transport_;
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
};

#endif
