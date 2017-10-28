/*
 *    Filename: main-generate-pix4d-geofile.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// SYSTEM
#include <memory>

// NON-SYSTEM
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-ortho/ortho-backward-grid.h>
#include <gflags/gflags.h>
#include <ros/ros.h>

DEFINE_string(util_data_directory, "", "");
DEFINE_string(util_filename_poses, "", "");
DEFINE_string(util_prefix_images, "", "");

int main(int  argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "util_generate_pix4d_geofile");
  ros::Time::init();

  // Parse input parameters.
  const std::string& base = FLAGS_util_data_directory;
  const std::string& filename_poses = FLAGS_util_filename_poses;
  const std::string& filename_images = base + FLAGS_util_prefix_images;

  // Load body poses from file.
  Poses T_G_Bs;
  const std::string& path_filename_poses = base + filename_poses;
  io::PoseFormat pose_format = io::PoseFormat::Standard;
  io::AerialMapperIO io_handler;
  io_handler.loadPosesFromFile(pose_format, path_filename_poses, &T_G_Bs);

  // Load images from file.
  size_t num_poses = T_G_Bs.size();
  Images images;
  io_handler.loadImagesFromFile(filename_images, num_poses, &images);

  // Export poses.
  io_handler.exportPix4dGeofile(T_G_Bs, images);

  return 0;
}
