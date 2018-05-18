/*
/*
 *    Filename: test-kd-tree.cc
 *  Created on: May 17, 2018
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// NON-SYSTEM
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "test_kd_tree");

  // kd Tree
  static constexpr size_t kMaxLeaf = 10u;
  static constexpr size_t kDimensionKdTree = 2u;
  typedef PointCloudAdaptor<PointCloud<double> > PC2KD;
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Adaptor<double, PC2KD>, PC2KD, kDimensionKdTree>
      my_kd_tree_t;
  PointCloud<double> cloud_kdtree_;
  std::unique_ptr<my_kd_tree_t> kd_tree_;
  std::unique_ptr<PC2KD> pc2kd_;

  // Add the sample points.
  cloud_kdtree_.pts.resize(6);
  cloud_kdtree_.pts[0].x = 0.0;
  cloud_kdtree_.pts[0].y = 0.0;
  cloud_kdtree_.pts[0].z = 0.0;

  cloud_kdtree_.pts[1].x = 1.0;
  cloud_kdtree_.pts[1].y = 0.0;
  cloud_kdtree_.pts[1].z = 0.0;

  cloud_kdtree_.pts[2].x = 2.0;
  cloud_kdtree_.pts[2].y = 0.0;
  cloud_kdtree_.pts[2].z = 0.0;

  cloud_kdtree_.pts[3].x = 3.0;
  cloud_kdtree_.pts[3].y = 0.0;
  cloud_kdtree_.pts[3].z = 5.0;

  cloud_kdtree_.pts[4].x = 4.0;
  cloud_kdtree_.pts[4].y = 0.0;
  cloud_kdtree_.pts[4].z = 5.0;

  cloud_kdtree_.pts[5].x = 5.0;
  cloud_kdtree_.pts[5].y = 0.0;
  cloud_kdtree_.pts[5].z = 0.0;

  pc2kd_.reset(new PC2KD(cloud_kdtree_));
  kd_tree_.reset(
      new my_kd_tree_t(kDimensionKdTree, *pc2kd_,
                       nanoflann::KDTreeSingleIndexAdaptorParams(kMaxLeaf)));
  kd_tree_->buildIndex();

  double search_radius = 0.5;
  std::vector<std::pair<int, double> > indices_dists;
  nanoflann::RadiusResultSet<double, int> result_set(
      search_radius, indices_dists);
  const double query_pt[3] = {2.5, 0.0, 0.0};
  kd_tree_->findNeighbors(result_set, query_pt, nanoflann::SearchParams());
  std::cout << "Query point x/y/z: " << query_pt[0] << "/" << query_pt[1] << "/"
            << query_pt[2] << " with search radius: " << search_radius << std::endl;
  for (const std::pair<int, double>& s : result_set.m_indices_dists) {
     std::cout << "ID: " << s.first << ", dist: " << s.second
               << ", x/y/z/: " << cloud_kdtree_.pts[s.first].x  << "/"
               <<  cloud_kdtree_.pts[s.first].y << "/"
               << cloud_kdtree_.pts[s.first].z << std::endl;
  }

  return 0;
}
