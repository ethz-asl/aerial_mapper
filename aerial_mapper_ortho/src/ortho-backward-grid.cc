/*
 *    Filename: ortho-backward-grid.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-ortho/ortho-backward-grid.h"

// SYSTEM
#include <fstream>
#include <iostream>
#include <math.h>

// NON-SYSTEM
#include <aerial-mapper-utils/utils-common.h>
#include <aslam/pipeline/undistorter.h>
#include <aslam/pipeline/undistorter-mapped.h>

namespace ortho {

OrthoBackwardGrid::OrthoBackwardGrid(
    const std::shared_ptr<aslam::NCamera> ncameras, const Settings& settings,
    grid_map::GridMap* map)
    : ncameras_(ncameras), settings_(settings) {
  CHECK(ncameras_);
  printParams();

  // Create one sample for every cell.
  samples_idx_range_.clear();
  size_t sample_counter = 0u;
  for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
    samples_idx_range_.push_back(sample_counter);
    map_sample_to_cell_index_.insert(std::make_pair(sample_counter, *it));
    ++sample_counter;
  }
}

void OrthoBackwardGrid::updateOrthomosaicLayer(const Poses& T_G_Cs,
                                               const Images& images,
                                               grid_map::GridMap* map) const {
  CHECK(ncameras_);
  const aslam::Camera& camera = ncameras_->getCamera(kFrameIdx);

  grid_map::Matrix& layer_ortho = (*map)["ortho"];
  grid_map::Matrix& layer_num_observations = (*map)["num_observations"];
  grid_map::Matrix& layer_elevation_angle = (*map)["elevation_angle"];
  const grid_map::Matrix& layer_elevation = (*map)["elevation"];
  grid_map::Matrix& layer_observation_index = (*map)["observation_index"];

  ros::Time time1 = ros::Time::now();
  for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map->getPosition(*it, position);
    const grid_map::Index index(*it);
    double x = index(0);
    double y = index(1);
    Eigen::Vector3d landmark_UTM =
        Eigen::Vector3d(position.x(), position.y(), layer_elevation(x, y));

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
        Eigen::Vector3d u = C_landmark;
        // Observation vector.
        double norm_u = sqrt(u(0) * u(0) + u(1) * u(1) + u(2) * u(2));
        // Angle (observation_in_camera, cell_center).
        double alpha = asin(std::fabs(u(2)) / norm_u);
        CHECK(alpha > 0.0);

        if (std::fabs(alpha) > layer_elevation_angle(x, y)) {
          layer_elevation_angle(x, y) = std::fabs(alpha);
          layer_observation_index(x, y) = i;
          layer_num_observations(x, y) += layer_num_observations(x, y);

          // Retrieve pixel intensity.
          const Eigen::Vector3d& C_landmark =
              T_G_Cs[i].inverse().transform(landmark_UTM);
          Eigen::Vector2d keypoint;
          camera.project3(C_landmark, &keypoint);
          const int kp_y = std::min(static_cast<int>(std::round(keypoint(1))),
                                    static_cast<int>(camera.imageHeight()) - 1);
          const int kp_x = std::min(static_cast<int>(std::round(keypoint(0))),
                                    static_cast<int>(camera.imageWidth()) - 1);
          const double gray_value = images[i].at<uchar>(kp_y, kp_x);

          // Update orthomosaic.
          layer_ortho(x, y) = gray_value;
        }  // if better observation angle
      }    // if visible
    }      // loop images
  }        // loop cells

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& delta_time = time2 - time1;
  LOG(INFO) << "Time elapsed = " << delta_time;
}

void OrthoBackwardGrid::process(const Poses& T_G_Bs, const Images& images,
                                grid_map::GridMap* map) const {
  CHECK(!T_G_Bs.empty());
  CHECK(T_G_Bs.size() == images.size());
  CHECK(map);
  LOG(INFO) << "Num. images = " << images.size();

  Poses T_G_Cs;
  for (const Pose& T_G_B : T_G_Bs) {
    T_G_Cs.push_back(T_G_B * ncameras_->get_T_C_B(0u).inverse());
  }
  updateOrthomosaicLayer(T_G_Cs, images, map);
}

void OrthoBackwardGrid::printParams() const {
  std::stringstream out;
  out << std::endl << std::string(50, '*') << std::endl
      << "Orthomosaic parameters:" << std::endl
      << utils::paramToString("Show orthomosaic opencv",
                              settings_.show_orthomosaic_opencv)
      << utils::paramToString("Save orthomosaic jpg",
                              settings_.save_orthomosaic_jpg)
      << utils::paramToString("Orthomosaic filename",
                              settings_.orthomosaic_jpg_filename)
//      << utils::paramToString("Use digital elevation map",
//                              settings_.use_digital_elevation_map)
      << std::string(50, '*') << std::endl;
  LOG(INFO) << out.str();
}

}  // namespace ortho
