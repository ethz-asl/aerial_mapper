/*
 *    Filename: dsm.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-dsm/dsm.h"

// NON-SYSTEM
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

namespace dsm {

Dsm::Dsm(const Settings& settings) : settings_(settings) {}

void Dsm::initializeAndFillKdTree(
    const Aligned<std::vector, Eigen::Vector3d>::type& point_cloud) {
  // Insert pointcloud in kdtree.
  cloud_kdtree_.pts.resize(point_cloud.size());
  LOG(INFO) << "Num points: " << point_cloud.size();
  for (size_t i = 0u; i < point_cloud.size(); ++i) {
    cloud_kdtree_.pts[i].x = point_cloud[i](0) - settings_.center_northing;
    cloud_kdtree_.pts[i].y = point_cloud[i](1) - settings_.center_easting;
    cloud_kdtree_.pts[i].z = point_cloud[i](2);
  }

  pc2kd_.reset(new PC2KD(cloud_kdtree_));
  kd_tree_.reset(
      new my_kd_tree_t(kDimensionKdTree, *pc2kd_,
                       nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf)));
  kd_tree_->buildIndex();
}

void Dsm::updateElevationLayer(grid_map::GridMap* map) {
  CHECK(map);
  for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map->getPosition(*it, position);

    double interp_radius = 5.0;
    std::vector<std::pair<int, double> > indices_dists;
    nanoflann::RadiusResultSet<double, int> resultSet(interp_radius,
                                                      indices_dists);
    const double query_pt[3] = {position.x(), position.y(), 0.0};
    kd_tree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams());

    if (true) {
      double lambda = 1.0;
      while (resultSet.size() == 0u) {
        nanoflann::RadiusResultSet<double, int> tmp(lambda * interp_radius,
                                                    indices_dists);
        kd_tree_->findNeighbors(tmp, query_pt, nanoflann::SearchParams());
        lambda *= 1.1;
        if (lambda * interp_radius > 7.0) {
          break;
        }
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
        heights.push_back(cloud_kdtree_.pts[s.first].z);
      }
      CHECK(distances.size() > 0u);
      CHECK(heights.size() > 0u);
      CHECK(distances.size() == heights.size());
      double idw_numerator = 0.0;
      double idw_denominator = 0.0;
      bool idw_perfect_match = false;
      for (size_t i = 0u; i < heights.size(); ++i) {
        if (!idw_perfect_match) {
          CHECK(distances[i] > 0.0);
          idw_numerator += heights[i] / (distances[i]);
          idw_denominator += 1.0 / (distances[i]);
        }
      }
      CHECK(idw_denominator > 0.0);
      double idw_height = idw_numerator / idw_denominator;
      map->at("elevation", *it) = idw_height;
    }
  }
}

void Dsm::process(
    const Aligned<std::vector, Eigen::Vector3d>::type& point_cloud,
    grid_map::GridMap* map) {
  CHECK(!point_cloud.empty());
  CHECK(map);
  initializeAndFillKdTree(point_cloud);
  updateElevationLayer(map);
}

void Dsm::printParams() {
  static constexpr int nameWidth = 30;
  std::cout << std::string(50, '*') << std::endl
            << "Starting Digital Elevation Map generation" << std::endl
            << std::left << std::setw(nameWidth)
            << " - Resolution: " << std::left << std::setw(nameWidth)
            << std::to_string(settings_.resolution) << std::endl << std::left
            << std::setw(nameWidth) << " - Interp. radius: " << std::left
            << std::setw(nameWidth)
            << std::to_string(settings_.interpolation_radius) << std::endl
            << std::left << std::setw(nameWidth)
            << " - Color Palette: " << std::left << std::setw(nameWidth)
            << std::to_string(settings_.color_palette) << std::endl;
  std::cout << std::string(50, '*') << std::endl;
}

}  // namespace dsm
