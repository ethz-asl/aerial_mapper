/*
 *    Filename: ortho-from-pcl.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// HEADER
#include "aerial-mapper-ortho/ortho-from-pcl.h"

// NON-SYSTEM
#include <aerial-mapper-utils/utils-common.h>

namespace ortho {

OrthoFromPcl::OrthoFromPcl(const Settings& settings) : settings_(settings) {
  printParams();
}

void OrthoFromPcl::process(
    const AlignedType<std::vector, Eigen::Vector3d>::type& pointcloud,
    const std::vector<int>& intensities, grid_map::GridMap* map) const {
  CHECK(!pointcloud.empty());
  CHECK(map);

  LOG(INFO) << "Number of points: " << pointcloud.size();
  PointCloud<double> cloud_kdtree;
  cloud_kdtree.pts.resize(pointcloud.size());
  for (size_t i = 0u; i < pointcloud.size(); ++i) {
    cloud_kdtree.pts[i].x = pointcloud[i](0);
    cloud_kdtree.pts[i].y = pointcloud[i](1);
    CHECK(i < intensities.size());
    cloud_kdtree.pts[i].z = double(intensities[i]);
  }

  // Construct a kd-tree index.
  typedef PointCloudAdaptor<PointCloud<double> > PC2KD;
  const PC2KD pc2kd(cloud_kdtree);
  const size_t kDimensionKdTree = 2u;
  const size_t kMaxLeaf = 10u;
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Adaptor<double, PC2KD>, PC2KD, kDimensionKdTree>
      my_kd_tree_t;
  my_kd_tree_t kd_tree(kDimensionKdTree, pc2kd,
                       nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf));
  kd_tree.buildIndex();

  // Loop over all cells.
  const ros::Time time1 = ros::Time::now();
  grid_map::Matrix& layer_ortho = (*map)["ortho"];
  for (grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map->getPosition(*it, position);
    const grid_map::Index index(*it);
    double x = index(0);
    double y = index(1);
    std::vector<std::pair<int, double> > indices_dists;
    nanoflann::RadiusResultSet<double, int> result_set(
        settings_.interpolation_radius, indices_dists);
    const double query_pt[3] = {position.x(), position.y(), 0.0};
    kd_tree.findNeighbors(result_set, query_pt, nanoflann::SearchParams());
    // Adaptive interpolation.
    if (settings_.use_adaptive_interpolation) {
      int lambda = 10;
      while (result_set.size() == 0u) {
        nanoflann::RadiusResultSet<double, int> tmp(
            lambda * settings_.interpolation_radius, indices_dists);
        kd_tree.findNeighbors(tmp, query_pt, nanoflann::SearchParams());
        lambda *= 10;
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
        heights.push_back(cloud_kdtree.pts[s.first].z);
      }
      CHECK(distances.size() > 0u);
      CHECK(heights.size() > 0u);
      CHECK(distances.size() == heights.size());
      double idw_numerator = 0.0;
      double idw_denominator = 0.0;
      bool idw_perfect_match = false;
      for (size_t i = 0u; i < heights.size(); ++i) {
        // Inverse distance weighing.
        if (distances[i] == 0.0) {
          // Perfect match, no interpolation needed.
          idw_numerator = heights[i];
          idw_denominator = 1.0;
          idw_perfect_match = true;
        }
        if (!idw_perfect_match) {
          CHECK(distances[i] > 0.0);
          idw_numerator += heights[i] / (distances[i]);
          idw_denominator += 1.0 / (distances[i]);
        }
      }
      // Inverse distance weighing.
      CHECK(idw_denominator > 0.0);
      const double idw_height = idw_numerator / idw_denominator;
      layer_ortho(x, y) = idw_height;
    }
  }

  const ros::Time time2 = ros::Time::now();
  const ros::Duration& delta_time = time2 - time1;
  LOG(INFO) << "Time elapsed: " << delta_time;
}

void OrthoFromPcl::printParams() const {
  std::stringstream out;
  out << std::endl << std::string(50, '*') << std::endl
      << "Ortho-From-Pcl parameters: " << std::endl
      << utils::paramToString("Show orthomosaic opencv",
                              settings_.show_orthomosaic_opencv)
      << utils::paramToString("Interp. radius", settings_.interpolation_radius)
      << utils::paramToString("Adapative interp.",
                              settings_.use_adaptive_interpolation)
      << utils::paramToString("Save orthomosaic jpg",
                              settings_.save_orthomosaic_jpg)
      << utils::paramToString("Orthomosaic jpg filename",
                              settings_.orthomosaic_jpg_filename)
      << std::string(50, '*') << std::endl;
  LOG(INFO) << out.str();
}

}  // namespace ortho
