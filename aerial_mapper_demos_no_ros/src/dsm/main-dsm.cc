/*
 *    Filename: main-dsm.cc
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

DEFINE_int32(dsm_color_palette, 0, "");
DEFINE_string(dsm_data_directory, "", "");
DEFINE_string(dsm_point_cloud_filename, "", "");
DEFINE_double(dsm_origin_easting_m, 0.0, "");
DEFINE_double(dsm_origin_elevation_m, 0.0, "");
DEFINE_double(dsm_origin_northing_m, 0.0, "");


int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  // TODO(hitimo): Remove ROS dependency here.
  ros::init(argc, argv, "dsm_from_file");

   // Parse input parameters.
  dsm::Settings settings;
  settings.color_palette = FLAGS_dsm_color_palette;
  settings.origin = Eigen::Vector3d(FLAGS_dsm_origin_easting_m,
                                    FLAGS_dsm_origin_northing_m,
                                    FLAGS_dsm_origin_elevation_m);
  const std::string& filename_point_cloud = FLAGS_dsm_data_directory +
      FLAGS_dsm_point_cloud_filename;

  io::AerialMapperIO io_handler;
  Aligned<std::vector, Eigen::Vector3d>::type point_cloud_xyz;
  LOG(INFO) << "Loading the point cloud from the file: "
            << filename_point_cloud;
  io_handler.loadPointCloudFromFile(filename_point_cloud,
                                    &point_cloud_xyz);

  LOG(INFO) << "Generating the digital surface map.";
  dsm::Dsm digital_surface_map(settings);
  digital_surface_map.process(point_cloud_xyz);

  return 0;
}
