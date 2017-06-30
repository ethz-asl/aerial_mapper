#include "aerial-mapper-ortho/ortho-forward-homography.h"
#include <opencv2/highgui/highgui.hpp>

FwOnlineOrthomosaic::FwOnlineOrthomosaic(
    std::string ncameras_yaml_path_filename) {
  prepareBlenderForNextImage();
  loadCameraRig(ncameras_yaml_path_filename);
  CHECK(ncameras_);
  undistorter_ =
      aslam::createMappedUndistorter(ncameras_->getCameraShared(0), 1.0, 1.0,
                                     aslam::InterpolationMethod::Linear);
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

  pub_ground_points_ = node_handle_.advertise<geometry_msgs::PolygonStamped>(
      "/orthomosaic/ground_points", 1000);
  image_transport_.reset(new image_transport::ImageTransport(node_handle_));
  pub_orthomosaic_image_ =
      image_transport_->advertise("/orthomosaic/result", 1);
  pub_undistorted_image_ =
      image_transport_->advertise("/orthomosaic/undistorted", 1);
}

void FwOnlineOrthomosaic::loadCameraRig(
    std::string ncameras_yaml_path_filename) {}

void FwOnlineOrthomosaic::addImage(cv::Mat image_warped_mutable) {
  if (image_warped_mutable.type() == CV_8U ||
      image_warped_mutable.type() == CV_8UC1) {
    cv::cvtColor(image_warped_mutable, image_warped_mutable, CV_GRAY2RGB);
  }
  if (image_warped_mutable.type() != CV_16SC3) {
    image_warped_mutable.convertTo(image_warped_mutable, CV_16SC3);
  }

  //  std::cout << result_mask_.size() << ", " << image_warped_mutable.size() <<
  //  std::endl;

  //  std::cout << result_mask_.type() << ", " << result_mask_.type() <<
  //  std::endl;
  //  cv::Mat mask_image;
  //  if (result_mask_.size() == image_warped_mutable.size()) {
  //    cv::Mat matD;
  //    cv::add(result_mask_, image_warped_mutable, matD);
  //   mask_image = matD > 10.0;
  //  } else {
  cv::Mat mask_image = (image_warped_mutable) > 0.1;
  //  }

  cv::cvtColor(mask_image, mask_image, CV_RGB2GRAY);

  //  if (result_mask_.size() == image_warped_mutable.size()) {
  //    mask_image = mask_image + result_mask_;
  //  }
  mask_image.convertTo(mask_image, CV_8U);
  CHECK(mask_image.type() == CV_8U);
  CHECK(blender_ != nullptr);
  //  cv::imshow("mask_image",mask_image);
  //  cv::waitKey(0);
  blender_->feed(image_warped_mutable.clone(), mask_image, cv::Point(0, 0));
}

void FwOnlineOrthomosaic::addImage(cv::Mat image_warped_mutable,
                                   cv::Mat mask_image) {
  if (image_warped_mutable.type() == CV_8U ||
      image_warped_mutable.type() == CV_8UC1) {
    cv::cvtColor(image_warped_mutable, image_warped_mutable, CV_GRAY2RGB);
  }
  if (image_warped_mutable.type() != CV_16SC3) {
    image_warped_mutable.convertTo(image_warped_mutable, CV_16SC3);
  }
  CHECK(mask_image.type() == CV_8U);
  CHECK(blender_ != nullptr);

  //  cv::imshow("mask_image2",mask_image);
  //  cv::waitKey(0);

  blender_->feed(image_warped_mutable.clone(), mask_image, cv::Point(0, 0));
}

void FwOnlineOrthomosaic::updateOrthomosaic(const aslam::Transformation& T_G_B,
                                            const cv::Mat& image) {
  cv::Mat image_undistorted;
  undistorter_->processImage(image, &image_undistorted);
  publishUndistortedImage(image_undistorted);

  geometry_msgs::PolygonStamped polygon;
  polygon.header.stamp = ros::Time::now();
  geometry_msgs::PolygonStamped polygon_stamped;
  polygon_stamped.header.stamp = ros::Time::now();
  polygon_stamped.header.frame_id = "/map";
  polygon_stamped.polygon.points.reserve(4);

  const aslam::Transformation& T_G_C =
      T_G_B * ncameras_->get_T_C_B(kFrameIdx).inverse();
  std::vector<cv::Point2f> ground_points, image_points;
  for (int border_pixel_index = 0;
       border_pixel_index < border_keypoints_.cols(); ++border_pixel_index) {
    Eigen::Vector3d C_ray;
    const Eigen::Vector2d& keypoint = border_keypoints_.col(border_pixel_index);
    ncameras_->getCameraShared(kFrameIdx)->backProject3(keypoint, &C_ray);
    const double scale = -(T_G_C.getPosition()(2) - 414) /
                         (T_G_C.getRotationMatrix() * C_ray)(2);
    const Eigen::Vector3d& G_landmark =
        T_G_C.getPosition() + scale * T_G_C.getRotationMatrix() * C_ray -
        Eigen::Vector3d(464980, 5.27226e+06, 414.087);
    ground_points.push_back(
        cv::Point2f(G_landmark(1) + 500.0, G_landmark(0) + 500.0));
    image_points.push_back(
        cv::Point2f(border_keypoints_.col(border_pixel_index)(0),
                    border_keypoints_.col(border_pixel_index)(1)));

    geometry_msgs::Point32 point;
    point.x = G_landmark(0);
    point.y = G_landmark(1);
    point.z = 0.0;
    polygon_stamped.polygon.points.push_back(point);
  }
  CHECK(ground_points.size() == 4);
  CHECK(image_points.size() == 4);
  pub_ground_points_.publish(polygon_stamped);

  const cv::Mat& perspective_transformation_matrix =
      cv::getPerspectiveTransform(image_points, ground_points);
  cv::Mat image_warped;
  cv::warpPerspective(image_undistorted, image_warped,
                      perspective_transformation_matrix, cv::Size(1000, 1000),
                      cv::INTER_NEAREST, cv::BORDER_CONSTANT);
  addImage(image_warped);
  //  std::cout << "blender_->blend start" << std::endl;
  blender_->blend(result_, result_mask_);
  //  std::cout << "blender_->blend done" << std::endl;
  //  cv::imshow("result_mask",result_mask_);
  //  cv::waitKey(0);
  //  std::cout << "prepareBlenderForNextImage start" << std::endl;
  prepareBlenderForNextImage();
  //  std::cout << "prepareBlenderForNextImage done" << std::endl;
  //  cv::waitKey(0);
  addImage(result_, result_mask_);
  showOrthomosaicCvWindow(result_);
  publishOrthomosaic(result_);
  cv::imwrite("/tmp/result.jpg", result_);
}

void FwOnlineOrthomosaic::batch(
    std::vector<kindr::minimal::QuatTransformation> T_G_Bs,
    std::vector<cv::Mat> images) {
  const ros::Time time1 = ros::Time::now();
  for (size_t i = 0u; i < images.size(); ++i) {
    cv::Mat image_undistorted;
    undistorter_->processImage(images[i], &image_undistorted);
    // publishUndistortedImage(image_undistorted);

    const aslam::Transformation& T_G_C =
        T_G_Bs[i] * ncameras_->get_T_C_B(kFrameIdx).inverse();
    std::vector<cv::Point2f> ground_points, image_points;
    for (int border_pixel_index = 0;
         border_pixel_index < border_keypoints_.cols(); ++border_pixel_index) {
      Eigen::Vector3d C_ray;
      const Eigen::Vector2d& keypoint =
          border_keypoints_.col(border_pixel_index);
      ncameras_->getCameraShared(kFrameIdx)->backProject3(keypoint, &C_ray);
      const double scale = -(T_G_C.getPosition()(2) - 414) /
                           (T_G_C.getRotationMatrix() * C_ray)(2);
      const Eigen::Vector3d& G_landmark =
          T_G_C.getPosition() + scale * T_G_C.getRotationMatrix() * C_ray -
          Eigen::Vector3d(464980, 5.27226e+06, 414.087);
      ground_points.push_back(
          cv::Point2f(G_landmark(1) + 500.0, G_landmark(0) + 500.0));
      image_points.push_back(
          cv::Point2f(border_keypoints_.col(border_pixel_index)(0),
                      border_keypoints_.col(border_pixel_index)(1)));
    }
    CHECK(ground_points.size() == 4);
    CHECK(image_points.size() == 4);
    const cv::Mat& perspective_transformation_matrix =
        cv::getPerspectiveTransform(image_points, ground_points);
    cv::Mat image_warped;
    cv::warpPerspective(image_undistorted, image_warped,
                        perspective_transformation_matrix, cv::Size(1000, 1000),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT);
    addImage(image_warped);
  }

  //  cv::Mat result, resultMask;
  //    blender->blend(result, resultMask);
  //    result.convertTo(result, (result.type() / 8) * 8);
  //  cv::imshow("Result", result);

  blender_->blend(result_, result_mask_);
  const ros::Time time2 = ros::Time::now();
  const ros::Duration& d1 = time2 - time1;
  std::cout << d1 << std::endl;

  //  prepareBlenderForNextImage();
  //  addImage(result_, result_mask_);
  showOrthomosaicCvWindow(result_);
  publishOrthomosaic(result_);

  //  cv::Mat mask;// = result_;
  //  cv::Mat m;
  //  std::cout << "mask.type = " << mask.type() << std::endl;
  //  mask.convertTo(m,CV_8U);
  //  std::cout << "mask.type = " << mask.type() << std::endl;
  //  // compute inverse thresholding (dark areas become "active" pixel in the
  //  mask) with OTSU thresholding:
  //  double grayThres = cv::threshold(result_, m, 0, 255, CV_THRESH_BINARY);
  //  std::cout << "mask.type = " << mask.type() << std::endl;
  //  // color all masked pixel red:
  cv::Mat input = result_;
  cv::Mat m = (result_) > 0;
  m = 255 - m;
  cv::imshow("m", m);
  cv::cvtColor(m, m, CV_RGB2GRAY);
  m.convertTo(m, CV_8U);
  CHECK(m.type() == CV_8U);

  std::cout << "mask.type = " << m.type() << std::endl;
  std::cout << "input.type = " << input.type() << std::endl;
  input.setTo(255, m);

  // compute median filter to remove the whitish black parts and darker white
  // parts

  // cv::imshow("input", input);
  cv::waitKey(0);

  cv::imwrite("/tmp/result.jpg", result_);
}

void FwOnlineOrthomosaic::showOrthomosaicCvWindow(cv::Mat current_mosaic) {
  current_mosaic.convertTo(current_mosaic, (current_mosaic.type() / 8) * 8);
  cv::imshow("Result", current_mosaic);
  cv::waitKey(1);
}

void FwOnlineOrthomosaic::prepareBlenderForNextImage() {
  blender_ = cv::detail::Blender::createDefault(cv::detail::Blender::FEATHER);
  cv::Rect rect(cv::Point(0, 0), cv::Point(1000, 1000));
  blender_->prepare(rect);
}

void FwOnlineOrthomosaic::showUndistortedCvWindow(cv::Mat image_undistorted) {
  cv::imshow("Undistorted image", image_undistorted);
  cv::waitKey(1);
}

void FwOnlineOrthomosaic::publishOrthomosaic(cv::Mat image) {
  // image.convertTo(image, (image.type() / 8) * 8);
  sensor_msgs::Image msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, image.rows,
                         image.cols, image.cols, image.data);
  pub_orthomosaic_image_.publish(msg);
  //  cv::imshow("Result", image);
  //  cv::waitKey(1);
}

void FwOnlineOrthomosaic::publishUndistortedImage(cv::Mat image) {
  sensor_msgs::Image msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, image.rows,
                         image.cols, image.cols, image.data);
  pub_undistorted_image_.publish(msg);
}
