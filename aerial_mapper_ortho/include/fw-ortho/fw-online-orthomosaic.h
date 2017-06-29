#ifndef FW_ONLINE_ORTHOMOSAIC_H_
#define FW_ONLINE_ORTHOMOSAIC_H_

#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <aslam/pipeline/undistorter-mapped.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/cameras/ncamera.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class FwOnlineOrthomosaic {
 public:
  FwOnlineOrthomosaic(std::string ncameras_yaml_path_filename );
  void updateOrthomosaic(const aslam::Transformation& T_G_B, const cv::Mat& image);
  void batch(std::vector<kindr::minimal::QuatTransformation> T_G_Bs,
             std::vector<cv::Mat> images);
 private:
  void addImage(cv::Mat image_warped, cv::Mat mask_image);
  void addImage(cv::Mat image_warped);
  void loadCameraRig(std::string ncameras_yaml_path_filename);
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
  ros::Publisher pub_ground_points_;
  std::unique_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Publisher pub_undistorted_image_;
  image_transport::Publisher pub_orthomosaic_image_;
};

#endif
