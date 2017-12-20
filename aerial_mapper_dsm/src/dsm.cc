/*
 *    Filename: dsm.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-dsm/dsm.h"

// SYSTEM
#include <iomanip>

// NON-SYSTEM
#include <aerial-mapper-utils/utils-common.h>
#include <glog/logging.h>

namespace dsm {

Dsm::Dsm(const Settings& settings, grid_map::GridMap* map)
    : settings_(settings) {
  printParams();
  if (settings_.use_multi_threads) {
    // Create one sample for every cell.
    samples_idx_range_.clear();
    size_t sample_counter = 0u;
    for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
      samples_idx_range_.push_back(sample_counter);
      map_sample_to_cell_index_.insert(std::make_pair(sample_counter, *it));
      ++sample_counter;
    }
  }
}

void Dsm::initializeAndFillKdTree(
    const AlignedType<std::vector, Eigen::Vector3d>::type& point_cloud) {
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
  const ros::Time time1 = ros::Time::now();
  for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map->getPosition(*it, position);
    std::vector<std::pair<int, double> > indices_dists;
    nanoflann::RadiusResultSet<double, int> result_set(
        settings_.interpolation_radius, indices_dists);
    const double query_pt[3] = {position.x(), position.y(), 0.0};
    kd_tree_->findNeighbors(result_set, query_pt, nanoflann::SearchParams());

    if (true) {
      double lambda = 1.0;
      while (result_set.size() == 0u) {
        nanoflann::RadiusResultSet<double, int> tmp(
            lambda * settings_.interpolation_radius, indices_dists);
        kd_tree_->findNeighbors(tmp, query_pt, nanoflann::SearchParams());
        lambda *= 1.1;
        if (lambda * settings_.interpolation_radius > 7.0) {
          break;
        }
      }
    }

    bool samples_in_interpolation_radius = result_set.size() > 0u;
    if (samples_in_interpolation_radius) {
      std::vector<double> distances;
      std::vector<double> heights;
      CHECK(result_set.size() > 0);
      distances.clear();
      heights.clear();
      for (const std::pair<int, double>& s : result_set.m_indices_dists) {
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
  const ros::Time time2 = ros::Time::now();
  const ros::Duration& delta_time = time2 - time1;
  VLOG(1) << "dt(update-dsm, single-thread): " << delta_time;
}

void Dsm::updateElevationLayerMultiThreaded(grid_map::GridMap* map) {
  CHECK(map);
  const ros::Time time1 = ros::Time::now();
  grid_map::Matrix& layer_elevation = (*map)["elevation"];

  auto generateCellWiseDsm = [&](const std::vector<size_t>& sample_idx_range_) {
    for (size_t sample_idx : sample_idx_range_) {
      grid_map::Index index = map_sample_to_cell_index_.at(sample_idx);
      double x = index(0);
      double y = index(1);

      grid_map::Position position;
      map->getPosition(index, position);

      std::vector<std::pair<int, double> > indices_dists;
      nanoflann::RadiusResultSet<double, int> result_set(
          settings_.interpolation_radius, indices_dists);
      const double query_pt[3] = {position.x(), position.y(), 0.0};
      kd_tree_->findNeighbors(result_set, query_pt, nanoflann::SearchParams());

      if (true) {
        double lambda = 1.0;
        while (result_set.size() == 0u) {
          nanoflann::RadiusResultSet<double, int> tmp(
              lambda * settings_.interpolation_radius, indices_dists);
          kd_tree_->findNeighbors(tmp, query_pt, nanoflann::SearchParams());
          lambda *= 1.1;
          if (lambda * settings_.interpolation_radius > 7.0) {
            break;
          }
        }
      }

      bool samples_in_interpolation_radius = result_set.size() > 0u;
      if (samples_in_interpolation_radius) {
        std::vector<double> distances;
        std::vector<double> heights;
        CHECK(result_set.size() > 0);
        distances.clear();
        heights.clear();
        for (const std::pair<int, double>& s : result_set.m_indices_dists) {
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
        layer_elevation(x, y) = idw_height;
      }
    }  // loop samples
  };   // lambda function

  const size_t num_samples = samples_idx_range_.size();
  const size_t num_threads = std::thread::hardware_concurrency();
  utils::parFor(num_samples, generateCellWiseDsm, num_threads);

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& delta_time = time2 - time1;
  VLOG(1) << "dt(update-dsm, multi-thread): " << delta_time;
}

void Dsm::process(
    const AlignedType<std::vector, Eigen::Vector3d>::type& point_cloud,
    grid_map::GridMap* map) {
  if (point_cloud.empty()) {
    LOG(WARNING) << "Passed empty point cloud to DSM module";
    return;
  }

  CHECK(map);
  initializeAndFillKdTree(point_cloud);
  if (settings_.use_multi_threads) {
    updateElevationLayerMultiThreaded(map);
  } else {
    updateElevationLayer(map);
  }
}

void Dsm::printParams() {
  std::stringstream out;
  out << std::endl << std::string(50, '*') << std::endl
      << "DSM parameters:" << std::endl
      << utils::paramToString("Interp. radius", settings_.interpolation_radius)
      << utils::paramToString("Adaptive interp.",
                              settings_.adaptive_interpolation)
      << utils::paramToString("Center easting", settings_.center_easting)
      << utils::paramToString("Center northing", settings_.center_northing)
      << std::string(50, '*') << std::endl;
  LOG(INFO) << out.str();
}

}  // namespace dsm
