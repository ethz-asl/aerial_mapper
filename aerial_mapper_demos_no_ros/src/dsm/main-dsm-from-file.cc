/*
 *    Filename: main-dsm-from-file.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */


// NON-SYSTEM
#include <aerial-mapper-dsm/dsm.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "dsm_from_file");

  io::AerialMapperIO io_handler;
  std::string filename_point_cloud = "/tmp/pointcloud.txt";
  Aligned<std::vector, Eigen::Vector3d>::type point_cloud_xyz;
  LOG(INFO) << "Loading the point cloud from the file: "
            << filename_point_cloud;
  io_handler.loadPointCloudFromFile(filename_point_cloud,
                                    &point_cloud_xyz);

  LOG(INFO) << "Generating the digital surface map.";
  dsm::Settings settings;
  settings.origin = Eigen::Vector3d(200.0, 200.0, 0.0);
  dsm::Dsm digital_surface_map(settings);
  digital_surface_map.process(point_cloud_xyz);

  return 0;
}
