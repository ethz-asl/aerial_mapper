/*
 *    Filename: ortho-backward-grid.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-ortho/ortho-backward-grid.h"

// NON-SYSTEM
#include <aerial-mapper-thirdparty/gps-conversions.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/pipeline/undistorter-mapped.h>

//DECLARE_string(orthomosaic_filename_poses);
//DECLARE_string(orthomosaic_image_directory);
//DECLARE_double(orthomosaic_easting_min);
//DECLARE_double(orthomosaic_northing_min);
//DECLARE_double(orthomosaic_easting_max);
//DECLARE_double(orthomosaic_northing_max);
//DECLARE_int32(orthomosaic_UTM_code);
//DECLARE_double(orthomosaic_resolution);
//DECLARE_string(orthomosaic_camera_calibration_yaml);
//DECLARE_string(orthomosaic_filename_height_map);

namespace ortho {

OrthoBackwardGrid::OrthoBackwardGrid(
    const std::shared_ptr<aslam::NCamera> ncameras, const Poses& T_G_Bs,
    const Images& images, const Eigen::Vector3d& origin)
    : ncameras_(ncameras), origin_(origin) {
  CHECK(ncameras_);
  // Transform to camera frame.
  Poses T_G_Cs;
  for (const Pose& T_G_B : T_G_Bs) {
    T_G_Cs.push_back(T_G_B * ncameras_->get_T_C_B(0u).inverse());
  }
  Images images_undistorted;
  std::unique_ptr<aslam::MappedUndistorter> undistorter_;
  undistorter_ =
      aslam::createMappedUndistorter(ncameras_->getCameraShared(0), 1.0, 1.0,
                                     aslam::InterpolationMethod::Linear);

  // TODO(hitimo): Undistort or not?
  Images images_new;
  for (const cv::Mat& image : images) {
    cv::Mat image_undistorted;
    undistorter_->processImage(image, &image_undistorted);
    images_undistorted.push_back(image_undistorted);
    images_new.push_back(image);
  }
  processBatch(T_G_Cs, images_new);
}

void OrthoBackwardGrid::processBatch(const Poses& T_G_Cs,
                                     const Images& images) {
  CHECK(!T_G_Cs.empty());
  CHECK(T_G_Cs.size() == images.size());
  LOG(INFO) << "images.size() = " << images.size();
  LOG(INFO) << "T_G_Cs.size() = " << T_G_Cs.size();

  printParams();
  const aslam::Camera& camera = ncameras_->getCamera(kFrameIdx);

  // TODO(hitimo): Re-enable the height map.
  bool use_digital_elevation_map = false;
  cv::Mat height_map;
  // loadHeightMap(FLAGS_orthomosaic_filename_height_map, &height_map);

  // Define the grid.
  const Eigen::Vector2d bottom_left = Eigen::Vector2d(
      settings_.orthomosaic_easting_min,
        settings_.orthomosaic_northing_min);
  const Eigen::Vector2d top_right = Eigen::Vector2d(
      settings_.orthomosaic_easting_max,
        settings_.orthomosaic_northing_max);
  const size_t width_east = std::fabs(bottom_left(0) - top_right(0));
  const size_t height_north = std::fabs(bottom_left(1) - top_right(1));
  const Eigen::Vector2d top_left =
      bottom_left + Eigen::Vector2d(0.0, height_north);

  // Iterate over all cells.
  const double d = static_cast<double>(settings_.orthomosaic_resolution);
  const Eigen::Vector2d& a = Eigen::Vector2d(0.5, -0.5) / d;
  const Eigen::Vector2d& b = a + top_left;
  cv::Mat orthomosaic(height_north * d, width_east * d, CV_8UC3,
                      cv::Scalar(255.0, 255.0, 255.0));
  Eigen::MatrixXi observation_map = Eigen::MatrixXi::Zero(
      static_cast<int>(width_east * d), static_cast<int>(height_north * d));

  size_t y_limit = static_cast<int>(height_north * d) - 1;
  size_t x_limit = static_cast<int>(width_east * d) - 1;
  const ros::Time time1 = ros::Time::now();
  for (size_t y = 0u; y < y_limit; ++y) {
    CHECK(observation_map.cols() > y);
    VLOG(100) << "[ " << y << " / " << height_north * d << " ]";
    for (size_t x = 0u; x < x_limit; ++x) {
      VLOG(200) << "[ " << x << " / " << width_east * d << " ]";
      CHECK(observation_map.rows() > x);
      const Eigen::Vector2d& cell_center =
          b +
          Eigen::Vector2d(static_cast<double>(x),
                          -static_cast<double>(y)) / d;
      if (use_digital_elevation_map) {
        // TODO(hitimo): Re-enable the height map.
        // const double height_from_dem = height_map.at<double>(y, x);
        // landmark_UTM =
        // Eigen::Vector3d(cell_center(0), cell_center(1),
        // height_from_dem);
      } else {
        const Eigen::Vector3d landmark_UTM(cell_center(0), cell_center(1),
                                           414.087);
        // Loop over all images.
        for (size_t i = 0u; i < images.size(); ++i) {
          const Eigen::Vector3d& C_landmark =
              T_G_Cs[i].inverse().transform(landmark_UTM);
          Eigen::Vector2d keypoint;
          const aslam::ProjectionResult& projection_result =
              camera.project3(C_landmark, &keypoint);

          // Check if keypoint visible.
          const bool keypoint_visible =
              (keypoint(0) >= 0.0) && (keypoint(1) >= 0.0) &&
              (keypoint(0) < static_cast<double>(camera.imageWidth())) &&
              (keypoint(1) < static_cast<double>(camera.imageHeight())) &&
              (projection_result.getDetailedStatus() !=
               aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
              (projection_result.getDetailedStatus() !=
               aslam::ProjectionResult::PROJECTION_INVALID);
          if (keypoint_visible) {
            // SIC! Order of keypoint x/y.
            const int kp_y =
                std::min(static_cast<int>(std::round(keypoint(1))), 479);
            const int kp_x =
                std::min(static_cast<int>(std::round(keypoint(0))), 751);
            const double gray_value = images[i].at<uchar>(kp_y, kp_x);
            orthomosaic.at<cv::Vec3b>(y, x) =
                cv::Vec3b(gray_value, gray_value, gray_value);
            break;
          }
        }
      }
    }
    cv::imshow("Orthomosaic", orthomosaic);
    cv::waitKey(1);
  }

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& d1 = time2 - time1;
  LOG(INFO) << "delta(t): " << d1 << std::endl;

  cv::imshow("Orthomosaic", orthomosaic);
  cv::imwrite("/tmp/orthomosaic_no_dem.jpeg", orthomosaic);
  cv::waitKey(0);
}

void OrthoBackwardGrid::processIncremental(const Poses& T_G_Cs,
                                           const Images& images) {
  CHECK(!T_G_Cs.empty());
  CHECK(T_G_Cs.size() == images.size());
  LOG(INFO) << "images.size() = " << images.size();
  LOG(INFO) << "T_G_Cs.size() = " << T_G_Cs.size();

  printParams();
  const aslam::Camera& camera = ncameras_->getCamera(kFrameIdx);

  // Define the grid.
  const Eigen::Vector2d bottom_left = Eigen::Vector2d(
      settings_.orthomosaic_easting_min,
        settings_.orthomosaic_northing_min);
  const Eigen::Vector2d top_right = Eigen::Vector2d(
      settings_.orthomosaic_easting_max,
        settings_.orthomosaic_northing_max);
  const size_t width_east = std::fabs(bottom_left(0) - top_right(0));
  const size_t height_north = std::fabs(bottom_left(1) - top_right(1));
  const Eigen::Vector2d top_left =
      bottom_left + Eigen::Vector2d(0.0, height_north);

  // bool use_digital_elevation_map = false;
  // Iterate over all cells.
  const double d = static_cast<double>(settings_.orthomosaic_resolution);
  const Eigen::Vector2d& a = Eigen::Vector2d(0.5, -0.5) / d;
  const Eigen::Vector2d& b = a + top_left;
  cv::Mat orthomosaic(height_north * d, width_east * d, CV_8UC3,
                      cv::Scalar(255.0, 255.0, 255.0));

  // Compute the ground points.
  Eigen::Matrix2Xd border_keypoints_;
  border_keypoints_.resize(Eigen::NoChange, 4);
  const size_t width = ncameras_->getCameraShared(0u)->imageWidth();
  const size_t height = ncameras_->getCameraShared(0u)->imageHeight();
  border_keypoints_.col(0) = Eigen::Vector2d(0.0, 0.0);
  border_keypoints_.col(1) =
      Eigen::Vector2d(static_cast<double>(width - 1u), 0.0);
  border_keypoints_.col(2) = Eigen::Vector2d(
        static_cast<double>(width - 1u), static_cast<double>(height - 1u));
  border_keypoints_.col(3) =
      Eigen::Vector2d(0.0, static_cast<double>(height - 1u));

  const ros::Time time1 = ros::Time::now();
  // Loop over all images.
  for (size_t i = 0u; i < images.size(); ++i) {
    const Pose& T_G_C = T_G_Cs[i];
    std::vector<cv::Point2f> ground_points, image_points;
    for (int border_pixel_index = 0;
         border_pixel_index < border_keypoints_.cols();
         ++border_pixel_index) {
      Eigen::Vector3d C_ray;
      const Eigen::Vector2d& keypoint =
          border_keypoints_.col(border_pixel_index);
      ncameras_->getCameraShared(kFrameIdx)->backProject3(keypoint, &C_ray);
      const double scale = -(T_G_C.getPosition()(2) - 414.087) /
                           (T_G_C.getRotationMatrix() * C_ray)(2);
      const Eigen::Vector3d& G_landmark =
          T_G_C.getPosition() + scale * T_G_C.getRotationMatrix() * C_ray;
      ground_points.push_back(cv::Point2f(G_landmark(0), G_landmark(1)));
      image_points.push_back(
          cv::Point2f(border_keypoints_.col(border_pixel_index)(0),
                      border_keypoints_.col(border_pixel_index)(1)));
    }

    double x_max, x_min, y_max, y_min;
    x_max = y_max = 0.0;
    x_min = y_min = 1.0e16;
    for (const cv::Point2f& ground_point : ground_points) {
      if (ground_point.x > x_max) {
        x_max = ground_point.x;
      }
      if (ground_point.x < x_min) {
        x_min = ground_point.x;
      }
      if (ground_point.y > y_max) {
        y_max = ground_point.y;
      }
      if (ground_point.y < y_min) {
        y_min = ground_point.y;
      }
    }

    // Find correponding grid points.
    int x_max_, x_min_, y_max_, y_min_;
    x_max_ = int(x_max) - b(0);
    x_min_ = int(x_min) - b(0);
    y_max_ = -(int(y_max) - b(1));
    y_min_ = -(int(y_min) - b(1));

    for (size_t x = x_min_; x <= x_max_; ++x) {
      for (size_t y = y_max_; y <= y_min_; ++y) {
        if (orthomosaic.at<cv::Vec3b>(y, x)[0] == 255 ||
            orthomosaic.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 255, 0)) {
          const Eigen::Vector2d& cell_center =
              b + Eigen::Vector2d(static_cast<double>(x),
                                  -static_cast<double>(y)) / d;
          const Eigen::Vector3d landmark_UTM(cell_center(0), cell_center(1),
                                             414.087);
          const Eigen::Vector3d& C_landmark =
              T_G_Cs[i].inverse().transform(landmark_UTM);
          Eigen::Vector2d keypoint;
          const aslam::ProjectionResult& projection_result =
              camera.project3(C_landmark, &keypoint);
          const bool keypoint_visible =
              (keypoint(0) >= 0.0) && (keypoint(1) >= 0.0) &&
              (keypoint(0) < static_cast<double>(camera.imageWidth())) &&
              (keypoint(1) < static_cast<double>(camera.imageHeight())) &&
              (projection_result.getDetailedStatus() !=
               aslam::ProjectionResult::POINT_BEHIND_CAMERA) &&
              (projection_result.getDetailedStatus() !=
               aslam::ProjectionResult::PROJECTION_INVALID);
          if (keypoint_visible) {
            const int kp_y =
                std::min(static_cast<int>(std::round(keypoint(1))), 479);
            const int kp_x =
                std::min(static_cast<int>(std::round(keypoint(0))), 751);
            const double gray_value = images[i].at<uchar>(kp_y, kp_x);
            orthomosaic.at<cv::Vec3b>(y, x) =
                cv::Vec3b(gray_value, gray_value, gray_value);
          } else {
            orthomosaic.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
          }
        }
      }
    }
  }  // images

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& d1 = time2 - time1;
  std::cout << d1 << std::endl;

  cv::imshow("Orthomosaic", orthomosaic);
  cv::waitKey(0);
  cv::imwrite("/tmp/orthomosaic_incremental.jpeg", orthomosaic);
}

//void OrthoBackwardGrid::printParams() {
//  const int nameWidth = 30;
//  std::cout << "***************************************************************"
//               "*******************************" << std::endl
//            << "Starting Orthomosaic image generation" << std::endl
//            << std::left
//            << std::setw(nameWidth) << " - Filename poses: " << std::left
//            << std::setw(nameWidth) << FLAGS_orthomosaic_filename_poses
//            << std::endl << std::left << std::setw(nameWidth)
//            << " - Image directory: " << std::left << std::setw(nameWidth)
//            << FLAGS_orthomosaic_image_directory << std::endl << std::left
//            << std::setw(nameWidth)
//            << " - Filename camera calibration: " << std::left
//            << std::setw(nameWidth)
//            << FLAGS_orthomosaic_camera_calibration_yaml
//            << std::endl << std::left << std::setw(nameWidth)
//            << " - Resolution: " << std::left << std::setw(nameWidth)
//            << FLAGS_orthomosaic_resolution << std::endl << std::left
//            << std::setw(nameWidth) << " - Easting min.: " << std::left
//            << std::setw(nameWidth) << FLAGS_orthomosaic_easting_min
//            << std::endl << std::left << std::setw(nameWidth)
//            << " - Easting max.: " << std::left << std::setw(nameWidth)
//            << FLAGS_orthomosaic_easting_max << std::endl << std::left
//            << std::setw(nameWidth) << " - Northing min.: " << std::left
//            << std::setw(nameWidth) << FLAGS_orthomosaic_northing_min
//            << std::endl << std::left << std::setw(nameWidth)
//            << " - Northing max.: " << std::left << std::setw(nameWidth)
//            << FLAGS_orthomosaic_northing_max << std::endl;
//  std::cout << "***************************************************************"
//               "*******************************" << std::endl;
//}

}  // namespace ortho
