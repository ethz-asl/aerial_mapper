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

DEFINE_bool(ortho_from_pcl_show_orthomosaic_opencv, true, "");
DEFINE_bool(ortho_from_pcl_use_adaptive_interpolation, false, "");
DEFINE_int32(ortho_from_pcl_interpolation_radius, 10, "");
DEFINE_string(ortho_from_pcl_data_directory, "", "");
DEFINE_string(ortho_from_pcl_orthomosaic_jpg_filename, "", "");
DEFINE_string(ortho_from_pcl_point_cloud_filename, "", "");
DEFINE_double(ortho_from_pcl_orthomosaic_resolution, 1.0, "");
DEFINE_double(ortho_from_pcl_orthomosaic_easting_min, 0.0, "");
DEFINE_double(ortho_from_pcl_orthomosaic_easting_max, 0.0, "");
DEFINE_double(ortho_from_pcl_orthomosaic_northing_min, 0.0, "");
DEFINE_double(ortho_from_pcl_orthomosaic_northing_max, 0.0, "");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  // TODO(hitimo): Remove ROS dependency here.
  ros::init(argc, argv, "ortho_from_pcl");

  // Parse input parameters.
  const std::string& filename_point_cloud =
      FLAGS_ortho_from_pcl_data_directory +
      FLAGS_ortho_from_pcl_point_cloud_filename;

  ortho::Settings settings;
  settings.interpolation_radius =
      FLAGS_ortho_from_pcl_interpolation_radius;
  settings.use_adaptive_interpolation =
      FLAGS_ortho_from_pcl_use_adaptive_interpolation;
  settings.show_orthomosaic_opencv =
      FLAGS_ortho_from_pcl_show_orthomosaic_opencv;
  settings.orthomosaic_jpg_filename =
      FLAGS_ortho_from_pcl_orthomosaic_jpg_filename;
  settings.orthomosaic_resolution =
      FLAGS_ortho_from_pcl_orthomosaic_resolution;
  settings.orthomosaic_easting_min =
      FLAGS_ortho_from_pcl_orthomosaic_easting_min;
  settings.orthomosaic_easting_max =
      FLAGS_ortho_from_pcl_orthomosaic_easting_max;
  settings.orthomosaic_northing_min =
      FLAGS_ortho_from_pcl_orthomosaic_northing_min;
  settings.orthomosaic_northing_max =
      FLAGS_ortho_from_pcl_orthomosaic_northing_max;

  // Load point cloud from file.
  io::AerialMapperIO io_handler;
  Aligned<std::vector, Eigen::Vector3d>::type point_cloud_xyz;
  std::vector<int> point_cloud_intensities;
  io_handler.loadPointCloudFromFile(filename_point_cloud,
                                    &point_cloud_xyz,
                                    &point_cloud_intensities);
  CHECK(point_cloud_xyz.size() > 0);
  CHECK(point_cloud_xyz.size() == point_cloud_intensities.size());

  // Generate the orthomosaic from the point cloud.
  ortho::OrthoFromPcl ortho(point_cloud_xyz,
                            point_cloud_intensities,
                            settings);

  return 0;
}
