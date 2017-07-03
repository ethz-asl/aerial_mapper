/*
 *    Filename: main_from_file.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// NON-SYSTEM
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

// PACKAGE
#include "aerial-mapper-dense-pcl/dense-pcl-planar-rectification.h"

namespace dense_pcl {

PlanarRectification::PlanarRectification(
    const std::shared_ptr<aslam::NCamera> ncameras, const Settings& settings)
    : first_frame_(true), settings_(settings), downsample_factor_(1.0),
      ncameras_(ncameras), node_handle_(),
      image_transport_(image_transport::ImageTransport(node_handle_)) {
  CHECK(ncameras_);

  // Camera intrinsis.
  aslam::PinholeCamera::ConstPtr pinhole_camera_ptr =
      std::dynamic_pointer_cast<const aslam::PinholeCamera>(
          ncameras_->getCameraShared(kFrameIdx));
  K_ = pinhole_camera_ptr->getCameraMatrix();
  LOG(INFO) << "Camera calibration matrix K = " << std::endl << K_;

  // Camera distortion parameters.
  Eigen::VectorXd k =
      ncameras_->getCameraShared(kFrameIdx)->getDistortion().getParameters();
  LOG(INFO) << "Camera distortion k = " << k.transpose();
  undistorter_.reset(
      new dense::Equidistant_Undistorter(K_, k(0), k(1), k(2), k(3)));
  T_B_C_ = ncameras_->get_T_C_B(kFrameIdx).inverse();
  LOG(INFO) << "Camera-IMU extrinsics T_B_C = " << std::endl
            << T_B_C_.getTransformationMatrix();


std::cout << "start advertising" << std::endl;
  pub_undistorted_image_ =
      image_transport_.advertise("/planar_rectification/undistorted", 1);
  std::cout << "end advertising" << std::endl;
  pub_point_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      "/planar_rectification/point_cloud", 1);

  // Create a planar rectification parameter object with default values
  dense::PlanarRectificationParams::Ptr parameters(
      new dense::PlanarRectificationParams);

  // Adapt the parameters according to the desired values
  node_handle_.param<bool>("showImages", parameters->showImages, false);
  node_handle_.param<bool>("allowShiftOfRectImgPlanes",
                           parameters->allowShiftOfRectImgPlanes, true);
  std::string matching_method;
  node_handle_.param<std::string>("matcher", matching_method, "STEREO_SGBM");
  if (matching_method == std::string("STEREO_BM")) {
    parameters->matcher = dense::MatchingMethods::STEREO_BM;
    ROS_INFO("Selected standard openCV blockmatcher.");
  } else if (matching_method == std::string("STEREO_BM")) {
    parameters->matcher = dense::MatchingMethods::STEREO_SGBM;
    ROS_INFO("Selected SGBM blockmatcher.");
  }
  node_handle_.param<int>("minDisparity", parameters->minDisp, 1);  // 10
  node_handle_.param<int>("numberOfDisparities",
                          parameters->numberOfDisparities, 80);
  node_handle_.param<int>("preFilterSizeBM", parameters->preFilterSizeBM, 9);
  node_handle_.param<int>("preFilterCapBM", parameters->preFilterCapBM, 31);
  node_handle_.param<int>("SADWindowSizeBM", parameters->SADWindowSizeBM, 15);
  node_handle_.param<int>("textureThresholdBM", parameters->textureThresholdBM,
                          20);
  node_handle_.param<int>("uniquenessRatioBM", parameters->uniquenessRatioBM,
                          80);
  node_handle_.param<int>("speckleWindowSizeBM",
                          parameters->speckleWindowSizeBM, 100);
  node_handle_.param<int>("speckleRangeBM", parameters->speckleRangeBM, 5);
  node_handle_.param<int>("SADWindowSizeSGBM", parameters->SADWindowSizeSGBM,
                          9);
  node_handle_.param<int>("P1SGBM", parameters->P1SGBM, 120);
  node_handle_.param<int>("P2SGBM", parameters->P2SGBM, 250);
  node_handle_.param<int>("disp12MaxDiffSGBM", parameters->disp12MaxDiffSGBM,
                          50);
  node_handle_.param<int>("preFilterCapSGBM", parameters->preFilterCapSGBM, 31);
  node_handle_.param<int>("uniquenessRatioSGBM",
                          parameters->uniquenessRatioSGBM, 10);
  node_handle_.param<int>("speckleWindowSizeSGBM",
                          parameters->speckleWindowSizeSGBM, 100);
  node_handle_.param<int>("speckleRangeSGBM", parameters->speckleRangeSGBM, 20);
  node_handle_.param<bool>("fullDPSGBM", parameters->fullDPSGBM,
                           true);  // false
  node_handle_.param<int>("parallelShift", parameters->parallelShift,
                          80);  // 10

  // Create a planar rectification object and initialize it with given
  // parameters.
  dense::OptimizerParams::Ptr optimizer_params(new dense::OptimizerParams);
  planar_rectification_.reset(new dense::PlanarRectification(parameters));

  node_handle_.param<bool>("showInvDepthMapImages_optimizer",
                           optimizer_params->showInvDepthMapImages, true);
  //  nodeLocal_.param<int>("windowSize_optimizer",
  //                        optimizer_params->windowSize);
  //  could_load_params &= nodeLocal_.getParam("useMedianFilter_optimizer",
  //                                         optimizer_params->useMedianFilter);
  //  could_load_params &= nodeLocal_.getParam("useMeanFilter_optimizer",
  //                                         optimizer_params->useMeanFilter);
  //  could_load_params &= nodeLocal_.getParam("sigma_disp_optimizer",
  //                                         optimizer_params->sigma_disp);
  //  could_load_params &= nodeLocal_.getParam("sigma_polRect_optimizer",
  // optimizer_params->sigma_polRect);

  int optimization_set_size = 20;
  dense::VFQueueParams::Ptr buffer_params(new dense::VFQueueParams);
  buffer_params->maxSize = optimization_set_size;
  buffer_params->minSizeToBeReady = optimization_set_size;  // Important!

  // Optimizer.
  optimizer_ = dense::Optimizer::Ptr(new dense::Optimizer(optimizer_params));
  buffer_ = dense::VFQueue::Ptr(new dense::VFQueue(buffer_params));
  ROS_ERROR_STREAM("Leaving ctor");
}

void PlanarRectification::addFrame(const aslam::Transformation& T_G_B,
                                   const cv::Mat& image_raw) {
  VLOG(0) << "add frame";
  const ros::Time time1 = ros::Time::now();
  if (first_frame_) {
    Eigen::Matrix3d K_new;
    cv::Mat image_undistorted;
    undistorter_->undistortImage(image_raw, downsample_factor_, &K_new,
                                 &image_undistorted);
    T_G_B_last_ = T_G_B;
    image_undistorted_last_ = image_undistorted;
    first_frame_ = false;
    return;
  }

  // Current frame.
  const aslam::Transformation& T_W_C = T_G_B * T_B_C_;
  const Eigen::Vector3d& c = T_W_C.getPosition();
  const Eigen::Matrix3d& R = T_W_C.getRotationMatrix();

  // Last frame.
  const aslam::Transformation& T_W_C_last = T_G_B_last_ * T_B_C_;
  const Eigen::Vector3d& c_last = T_W_C_last.getPosition();
  const Eigen::Matrix3d& R_last = T_W_C_last.getRotationMatrix();

  //  std::cout << "c = " << c.transpose() << ", c_last = " <<
  //  c_last.transpose() << std::endl;
  //  std::cout << "R = " << std::endl<< R << std::endl <<", R_last = "
  //            << std::endl<< R_last.transpose() << std::endl;

  // Create a StereoPair
  Eigen::Matrix3d K_new;
  cv::Mat image_undistorted;
  undistorter_->undistortImage(image_raw, downsample_factor_, &K_new,
                               &image_undistorted);

  //  std::cout << "cv types: " << image_undistorted_last_.type() << ", "
  //            << image_undistorted.type() << std::endl;
  dense::StereoPair::Ptr stereoPair(
      new dense::StereoPair(K_, image_undistorted_last_, true, c_last, R_last,
                            K_, image_undistorted, true, c, R));

  cv::imshow("image_undistorted_last", image_undistorted_last_);
  cv::imshow("image_undistorted", image_undistorted);
  cv::waitKey(0);

  planar_rectification_->densifyStereoPair(*stereoPair);
  if (stereoPair->vF1->isDensified()) {
    // Get the point cloud (defined in the world system)
    cv::Mat_<cv::Vec3f> point_cloud;
    cv::Mat point_cloud_intensity;
    stereoPair->vF1->getPointCloud(stereoPair->vF1->c_, stereoPair->vF1->R_W_C_,
                                   point_cloud, point_cloud_intensity);
    const ros::Time time2 = ros::Time::now();
    const ros::Duration& d1 = time2 - time1;
    std::cout << d1 << std::endl;

    // Compute the average scene depth
    float zAvg = 100;
    stereoPair->vF1->getAvgSceneDepth(zAvg);

    // Publish point cloud
    sensor_msgs::PointCloud2 point_cloud_msg;
    ros::Time timestamp = ros::Time::now();
    Utils::convertCvPclToRosPCL2Msg(point_cloud, point_cloud_intensity,
                                    "/world", timestamp, point_cloud_msg);
    //pub_point_cloud_.publish(point_cloud_msg);

    //     pcl::PointCloud<pcl::PointXYZ> cloud;
    //     pcl::fromROSMsg(point_cloud_msg, cloud);
    //     for (size_t i = 0u; i < cloud.size(); ++i) {
    //       std::cout << Eigen::Vector3d(cloud[i].x, cloud[i].y,
    //       cloud[i].z).transpose() << std::endl;
    //     }

    // Compute the angle between the baseline and the optical axis of each
    // vision frame of the stereo pair.
    // (Interesting for the freature "shift of the rectified image planes!")
    Eigen::Vector3d baseline = stereoPair->vF1->c_ - stereoPair->vF2->c_;
    baseline.normalize();
    Eigen::Vector3d optical_axis_vF1 = stereoPair->vF1->R_W_C_.col(2);
    Eigen::Vector3d optical_axis_vF2 = stereoPair->vF2->R_W_C_.col(2);
    optical_axis_vF1.normalize();
    optical_axis_vF2.normalize();
    float angle_1 =
        (std::acos(baseline.dot(optical_axis_vF1)) * 360) / (2 * M_PI);
    float angle_2 =
        (std::acos(baseline.dot(optical_axis_vF2)) * 360) / (2 * M_PI);

    std::stringstream ss;
    ss << ">> Stereo Pair Info:" << std::endl
       << "Image Size: " << stereoPair->vF1->image_.size() << std::endl
       << "Baseline  : " << stereoPair->baseline << " m" << std::endl
       << "Angle between optical axis of vision frame 1 and baseline : "
       << angle_1 << " °" << std::endl
       << "Angle between optical axis of vision frame 2 and baseline : "
       << angle_2 << " °" << std::endl << "Angle between optical axes: "
       << (std::acos(stereoPair->dotOpticalAxes) * 360.0) / (2.0 * M_PI) << " °"
       << std::endl << "Epipole1: " << std::endl << stereoPair->e1.hnormalized()
       << std::endl << "Epipole2: " << std::endl << stereoPair->e2.hnormalized()
       << std::endl << std::endl
       << ">> Rectification And Densification:" << std::endl
       << "Avg Scene Depth: " << zAvg << " m" << std::endl;
    VLOG(0) << ss.str();

    CHECK(buffer_);
    buffer_->push(stereoPair->vF1);
  }

  //  std::vector<dense::VisionFrame::Ptr> optimization_set;
  //  for (std::deque<dense::VisionFrame::Ptr>::iterator i =
  //  buffer_->getBeginIterator();
  //       i != buffer_->getEndIterator(); ++i) {
  //    optimization_set.push_back(*i);
  //  }

  //  dense::VisionFrame::Ptr fused_vision_frame;
  //  bool is_vision_frame_fused =
  //      optimizer_->fuseVisionFrameSet(optimization_set, fused_vision_frame);
  //  float d;
  //  fused_vision_frame->getAvgSceneDepth(d);

  //  sensor_msgs::PointCloud2 point_cloud_msg;
  //  ros::Time timestamp = ros::Time::now();
  //  cv::Mat_<cv::Vec3f> point_cloud;
  //  cv::Mat point_cloud_intensity;
  //  fused_vision_frame->getPointCloud(stereoPair->vF1->c_,
  //  stereoPair->vF1->R_W_C_,
  //                                 point_cloud, point_cloud_intensity);

  //  //fused_vision_frame->getPointCloud(point_cloud, point_cloud_intensity);
  //  Utils::convertCvPclToRosPCL2Msg(point_cloud, point_cloud_intensity,
  //  "/world",
  //                                  timestamp, point_cloud_msg);
  // pub_point_cloud_.publish(point_cloud_msg);

  T_G_B_last_ = T_G_B;
  image_undistorted_last_ = image_undistorted;
}

void PlanarRectification::showUndistortedCvWindow(
    const cv::Mat& image_undistorted) {
  cv::imshow("Undistorted image", image_undistorted);
  cv::waitKey(1);
}

void PlanarRectification::publishUndistortedImage(const cv::Mat& image) {
  sensor_msgs::Image msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, image.rows,
                         image.cols, image.cols, image.data);
  //pub_undistorted_image_.publish(msg);
}

}  // namespace dense_pcl
