/*
 *    Filename: ortho-from-pcl.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-ortho/ortho-from-pcl.h"

// NON-SYSTEM
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

namespace ortho {

OrthoFromPcl::OrthoFromPcl(
    const Aligned<std::vector, Eigen::Vector3d>::type& pointcloud,
    const std::vector<int>& intensities, const Settings& settings)
    : settings_(settings) {
  process(pointcloud, intensities);
}

void OrthoFromPcl::process(
    const Aligned<std::vector, Eigen::Vector3d>::type& pointcloud,
    const std::vector<int>& intensities) {
  CHECK(!pointcloud.empty());

  LOG(INFO) << "Number of points: " << pointcloud.size();
  PointCloud<double> cloud_kdtree;
  cloud_kdtree.pts.resize(pointcloud.size());
  double min_height = std::numeric_limits<double>::max();
  double max_height = std::numeric_limits<double>::min();
  for (size_t i = 0u; i < pointcloud.size(); ++i) {
    cloud_kdtree.pts[i].x = pointcloud[i](0);
    cloud_kdtree.pts[i].y = pointcloud[i](1);
    CHECK(i < intensities.size());
    cloud_kdtree.pts[i].z = double(intensities[i]);

    // Save boundaries.
    if (cloud_kdtree.pts[i].z < min_height) {
      min_height = cloud_kdtree.pts[i].z;
    }
    if (cloud_kdtree.pts[i].z > max_height) {
      max_height = cloud_kdtree.pts[i].z;
    }
  }
  VLOG(10) << "Minimum height = " << min_height;
  VLOG(10) << "Maximum height = " << max_height;

  VLOG(100) << "Construct a kd-tree index.";
  typedef PointCloudAdaptor<PointCloud<double> > PC2KD;
  const PC2KD pc2kd(cloud_kdtree);
  const size_t kDimensionKdTree = 2u;
  const size_t kMaxLeaf = 10u;
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Adaptor<double, PC2KD>, PC2KD, kDimensionKdTree>
      my_kd_tree_t;
  my_kd_tree_t index(kDimensionKdTree, pc2kd,
                     nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf));
  index.buildIndex();

  VLOG(100) << "Define the grid.";
  const Eigen::Vector2d bottom_left(settings_.orthomosaic_easting_min,
                                    settings_.orthomosaic_northing_min);
  Eigen::Vector2d top_right(settings_.orthomosaic_northing_min,
                            settings_.orthomosaic_northing_max);
  const size_t width_east = std::fabs(bottom_left(0) - top_right(0));
  const size_t height_north = std::fabs(bottom_left(1) - top_right(1));
  const Eigen::Vector2d top_left =
      bottom_left + Eigen::Vector2d(0.0, height_north);

  const double d = static_cast<double>(settings_.orthomosaic_resolution);
  cv::Mat ortho_image_idw(height_north * d, width_east * d, CV_8UC3,
                          cv::Scalar(255, 255, 255));
  double height_max = max_height;
  double height_min = min_height;
  VLOG(100) << "Loop over all cells.";
  common::ProgressBar progress_bar(static_cast<size_t>(height_north * d));
  const ros::Time time1 = ros::Time::now();
  for (size_t y = 0u; y < static_cast<size_t>(height_north * d); ++y) {
    progress_bar.increment();
    for (size_t x = 0u; x < static_cast<size_t>(width_east * d); ++x) {
      CHECK(x < width_east * d);
      CHECK(y < height_north * d);
      const Eigen::Vector2d cell_center =
          top_left +
          Eigen::Vector2d(static_cast<double>(x),
                          -static_cast<double>(y)) / d +
          Eigen::Vector2d(0.5, -0.5) / d;
      std::vector<std::pair<int, double> > indices_dists;
      nanoflann::RadiusResultSet<double, int> resultSet(
          settings_.interpolation_radius, indices_dists);
      const double query_pt[3] = {cell_center(0), cell_center(1), 0.0};
      index.findNeighbors(resultSet, query_pt, nanoflann::SearchParams());
      // Adaptive interpolation.
      if (settings_.use_adaptive_interpolation) {
        int lambda = 10;
        while (resultSet.size() == 0u) {
          nanoflann::RadiusResultSet<double, int> tmp(
              lambda * settings_.interpolation_radius, indices_dists);
          index.findNeighbors(tmp, query_pt, nanoflann::SearchParams());
          lambda *= 10;
        }
      }
      bool samples_in_interpolation_radius = resultSet.size() > 0u;
      if (samples_in_interpolation_radius) {
        std::vector<double> distances;
        std::vector<double> heights;
        CHECK(resultSet.size() > 0);
        distances.clear();
        heights.clear();
        for (const std::pair<int, double>& s : resultSet.m_indices_dists) {
          distances.push_back(s.second);
          heights.push_back(cloud_kdtree.pts[s.first].z);
        }
        CHECK(distances.size() > 0u);
        CHECK(heights.size() > 0u);
        CHECK(distances.size() == heights.size());
        double idw_top = 0.0;
        double idw_bottom = 0.0;
        bool idw_perfect_match = false;
        for (size_t i = 0u; i < heights.size(); ++i) {
          // Inverse distance weighting.
          if (distances[i] == 0.0) {
            // Perfect match, no interpolation needed.
            idw_top = heights[i];
            idw_bottom = 1.0;
            idw_perfect_match = true;
          }
          if (!idw_perfect_match) {
            CHECK(distances[i] > 0.0);
            idw_top += heights[i] / (distances[i]);
            idw_bottom += 1.0 / (distances[i]);
          }
        }
        // Inverse distance weighting.
        CHECK(idw_bottom > 0.0);
        const double idw_height = idw_top / idw_bottom;
        const double idw_scaled_height =
            (255.0) / (height_max - height_min) * idw_height -
            (255.0) / (height_max - height_min) * height_min;
        ortho_image_idw.at<cv::Vec3b>(y, x) =
            cv::Vec3b(idw_scaled_height,
                      idw_scaled_height, idw_scaled_height);
      } else {
        ortho_image_idw.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
      }
    }
  }

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& delta_time = time2 - time1;
  LOG(INFO) << "Time elapsed: " << delta_time;

  if (settings_.show_orthomosaic_opencv) {
    cv::imshow("ortho_from_pcl", ortho_image_idw);
    cv::waitKey(0);
  }
  if (settings_.save_orthomosaic_jpg) {
    cv::imwrite(settings_.orthomosaic_jpg_filename, ortho_image_idw);
  }
}

}  // namespace ortho
