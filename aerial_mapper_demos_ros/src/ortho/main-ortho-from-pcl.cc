/*
 *    Filename: main-ortho-from-pcl.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// NON-SYSTEM
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-ortho/ortho-from-pcl.h>
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "ortho_from_pcl");
  ros::NodeHandle nh;

  // Load point cloud from file.
  io::AerialMapperIO io_handler;
  std::string filename_point_cloud = "/tmp/pointcloud.txt";
  Aligned<std::vector, Eigen::Vector3d>::type point_cloud_xyz;
  std::vector<int> point_cloud_intensities;
  io_handler.loadPointCloudFromFile(filename_point_cloud,
                                    &point_cloud_xyz,
                                    &point_cloud_intensities);

  // Generate the orthomosaic from the point cloud.
  ortho::Settings settings;
  ortho::OrthoFromPcl ortho(point_cloud_xyz,
                            point_cloud_intensities,
                            settings);

  return 0;
}
