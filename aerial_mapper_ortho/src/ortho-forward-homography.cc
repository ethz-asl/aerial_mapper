/*
 *    Filename: ortho-forward-homography.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-ortho/ortho-forward-homography.h"

// NON-SYSTEM
#include <opencv2/highgui/highgui.hpp>

namespace ortho {

OrthoForwardHomography::OrthoForwardHomography(
    const std::shared_ptr<aslam::NCamera>& ncameras, const Settings& settings)
    : ncameras_(ncameras),
      settings_(settings),
      node_handle_(),
      image_transport_(image_transport::ImageTransport(node_handle_)),
      pub_orthomosaic_image_(
          image_transport_.advertise("/orthomosaic/result", 1)),
      pub_undistorted_image_(
          image_transport_.advertise("/orthomosaic/undistorted", 1)) {
  CHECK(ncameras_);
  prepareBlenderForNextImage();
  undistorter_ = aslam::createMappedUndistorter(
      ncameras_->getCamera(0), 1.0, 1.0, aslam::InterpolationMethod::Linear);
  border_keypoints_.resize(Eigen::NoChange, 4);
  const size_t width = ncameras_->getCameraShared(0u)->imageWidth();
  const size_t height = ncameras_->getCameraShared(0u)->imageHeight();
  border_keypoints_.col(0) = Eigen::Vector2d(0.0, 0.0);
  border_keypoints_.col(1) =
      Eigen::Vector2d(static_cast<double>(width - 1u), 0.0);
  border_keypoints_.col(2) = Eigen::Vector2d(static_cast<double>(width - 1u),
                                             static_cast<double>(height - 1u));
  border_keypoints_.col(3) =
      Eigen::Vector2d(0.0, static_cast<double>(height - 1u));
}

void OrthoForwardHomography::addImage(cv::Mat image_warped_mutable) {
  if (image_warped_mutable.type() == CV_8U ||
      image_warped_mutable.type() == CV_8UC1) {
    cv::cvtColor(image_warped_mutable, image_warped_mutable, CV_GRAY2RGB);
  }
  if (image_warped_mutable.type() != CV_16SC3) {
    image_warped_mutable.convertTo(image_warped_mutable, CV_16SC3);
  }
  cv::Mat mask_image = (image_warped_mutable) > 0.1;
  cv::cvtColor(mask_image, mask_image, CV_RGB2GRAY);

  mask_image.convertTo(mask_image, CV_8U);
  CHECK_EQ(mask_image.type(), CV_8U);
  CHECK(blender_ != nullptr);
  blender_->feed(image_warped_mutable.clone(), mask_image, cv::Point(0, 0));
}

void OrthoForwardHomography::addImage(cv::Mat image_warped_mutable,
                                      cv::Mat mask_image) {
  if (image_warped_mutable.type() == CV_8U ||
      image_warped_mutable.type() == CV_8UC1) {
    cv::cvtColor(image_warped_mutable, image_warped_mutable, CV_GRAY2RGB);
  }
  if (image_warped_mutable.type() != CV_16SC3) {
    image_warped_mutable.convertTo(image_warped_mutable, CV_16SC3);
  }
  CHECK_EQ(mask_image.type(), CV_8U);
  CHECK(blender_ != nullptr);

  blender_->feed(image_warped_mutable.clone(), mask_image, cv::Point(0, 0));
}

void OrthoForwardHomography::updateOrthomosaic(const Pose& T_G_B,
                                               const cv::Mat& image) {
  cv::Mat image_undistorted;
  undistorter_->processImage(image, &image_undistorted);
  publishUndistortedImage(image_undistorted);

  geometry_msgs::PolygonStamped polygon;
  polygon.header.stamp = ros::Time::now();

  const aslam::Transformation& T_G_C =
      T_G_B * ncameras_->get_T_C_B(kFrameIdx).inverse();
  std::vector<cv::Point2f> ground_points, image_points;
  for (int border_pixel_index = 0;
       border_pixel_index < border_keypoints_.cols(); ++border_pixel_index) {
    Eigen::Vector3d C_ray;
    const Eigen::Vector2d& keypoint = border_keypoints_.col(border_pixel_index);
    ncameras_->getCameraShared(kFrameIdx)->backProject3(keypoint, &C_ray);
    const double scale =
        -(T_G_C.getPosition()(2) - settings_.ground_plane_elevation_m) /
        (T_G_C.getRotationMatrix() * C_ray)(2);
    const Eigen::Vector3d& G_landmark =
        T_G_C.getPosition() + scale * T_G_C.getRotationMatrix() * C_ray -
        settings_.origin;
    ground_points.push_back(cv::Point2f(
        G_landmark(1) +
            static_cast<double>(settings_.width_mosaic_pixels) / 2.0,
        G_landmark(0) +
            static_cast<double>(settings_.height_mosaic_pixels) / 2.0));
    image_points.push_back(
        cv::Point2f(border_keypoints_.col(border_pixel_index)(0),
                    border_keypoints_.col(border_pixel_index)(1)));
  }
  CHECK_EQ(ground_points.size(), 4u);
  CHECK_EQ(image_points.size(), 4u);

  const cv::Mat& perspective_transformation_matrix =
      cv::getPerspectiveTransform(image_points, ground_points);
  cv::Mat image_warped;
  cv::warpPerspective(
      image_undistorted, image_warped, perspective_transformation_matrix,
      cv::Size(settings_.width_mosaic_pixels, settings_.height_mosaic_pixels),
      cv::INTER_NEAREST, cv::BORDER_CONSTANT);
  addImage(image_warped);
  blender_->blend(result_, result_mask_);
  prepareBlenderForNextImage();
  addImage(result_, result_mask_);
  showOrthomosaicCvWindow(result_);
  publishOrthomosaic(result_);

  if (ksaveOrthoIncrementallyToFile) {
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << ++image_idx;
    cv::imwrite("/tmp/image_" + ss.str() + ".jpeg", result_);
  } else {
    cv::imwrite(settings_.filename_mosaic_output, result_);
  }

  ros::spinOnce();
}

void OrthoForwardHomography::batch(const Poses& T_G_Bs, const Images& images) {
  for (size_t i = 0u; i < images.size(); ++i) {
    cv::Mat image_undistorted;
    undistorter_->processImage(images[i], &image_undistorted);

    const aslam::Transformation& T_G_C =
        T_G_Bs[i] * ncameras_->get_T_C_B(kFrameIdx).inverse();
    std::vector<cv::Point2f> ground_points, image_points;
    for (int border_pixel_index = 0;
         border_pixel_index < border_keypoints_.cols(); ++border_pixel_index) {
      Eigen::Vector3d C_ray;
      const Eigen::Vector2d& keypoint =
          border_keypoints_.col(border_pixel_index);
      ncameras_->getCameraShared(kFrameIdx)->backProject3(keypoint, &C_ray);
      const double scale =
          -(T_G_C.getPosition()(2) - settings_.ground_plane_elevation_m) /
          (T_G_C.getRotationMatrix() * C_ray)(2);
      const Eigen::Vector3d& G_landmark =
          T_G_C.getPosition() + scale * T_G_C.getRotationMatrix() * C_ray -
          settings_.origin;
      ground_points.push_back(cv::Point2f(
          G_landmark(1) +
              static_cast<double>(settings_.width_mosaic_pixels) / 2.0,
          G_landmark(0) +
              static_cast<double>(settings_.width_mosaic_pixels) / 2.0));
      image_points.push_back(
          cv::Point2f(border_keypoints_.col(border_pixel_index)(0),
                      border_keypoints_.col(border_pixel_index)(1)));
    }
    CHECK_EQ(ground_points.size(), 4u);
    CHECK_EQ(image_points.size(), 4u);
    const cv::Mat& perspective_transformation_matrix =
        cv::getPerspectiveTransform(image_points, ground_points);
    cv::Mat image_warped;
    cv::warpPerspective(
        image_undistorted, image_warped, perspective_transformation_matrix,
        cv::Size(settings_.width_mosaic_pixels, settings_.height_mosaic_pixels),
        cv::INTER_NEAREST, cv::BORDER_CONSTANT);
    addImage(image_warped);
  }

  blender_->blend(result_, result_mask_);
  showOrthomosaicCvWindow(result_);
  // publishOrthomosaic(result_);

  // Compute the mask.
  cv::Mat m = (result_) > 0;
  m = 255 - m;
  cv::cvtColor(m, m, CV_RGB2GRAY);
  m.convertTo(m, CV_8U);
  CHECK_EQ(m.type(), CV_8U);

  // Set color of "unobserved pixels" (0 for black, 255 for white).
  result_.setTo(0, m);
  cv::imwrite(settings_.filename_mosaic_output, result_);
}

void OrthoForwardHomography::showOrthomosaicCvWindow(cv::Mat current_mosaic) {
  current_mosaic.convertTo(current_mosaic, (current_mosaic.type() / 8) * 8);
  cv::imshow("Result", current_mosaic);
  cv::waitKey(1);
}

void OrthoForwardHomography::prepareBlenderForNextImage() {
  blender_ = cv::detail::Blender::createDefault(cv::detail::Blender::FEATHER);
  cv::Rect rect(cv::Point(0, 0), cv::Point(settings_.width_mosaic_pixels,
                                           settings_.height_mosaic_pixels));
  blender_->prepare(rect);
}

void OrthoForwardHomography::showUndistortedCvWindow(
    cv::Mat image_undistorted) {
  cv::imshow("Undistorted image", image_undistorted);
  cv::waitKey(1);
}

void OrthoForwardHomography::publishOrthomosaic(cv::Mat image) {
  sensor_msgs::Image msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = settings_.nframe_id;
  sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, image.rows,
                         image.cols, image.cols, image.data);
  pub_orthomosaic_image_.publish(msg);
  ros::spinOnce();
}

void OrthoForwardHomography::publishUndistortedImage(cv::Mat image) {
  sensor_msgs::Image msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = settings_.nframe_id;
  sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, image.rows,
                         image.cols, image.cols, image.data);
  pub_undistorted_image_.publish(msg);
  ros::spinOnce();
}

}  // namespace ortho
