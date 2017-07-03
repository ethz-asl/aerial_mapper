/*
 *    Filename: main-ortho-backward-grid.cc
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

DEFINE_string(backward_grid_data_directory, "", "");
DEFINE_string(backward_grid_filename_poses, "", "");
DEFINE_string(backward_grid_prefix_images, "", "");
DEFINE_string(backward_grid_filename_camera_rig, "", "");
DEFINE_double(backward_grid_orthomosaic_easting_min, 0.0, "");
DEFINE_double(backward_grid_orthomosaic_easting_max, 0.0, "");
DEFINE_double(backward_grid_orthomosaic_northing_min, 0.0, "");
DEFINE_double(backward_grid_orthomosaic_northing_max, 0.0, "");
DEFINE_double(backward_grid_orthomosaic_resolution, 0.0, "");
DEFINE_bool(backward_grid_show_orthomosaic_opencv, true, "");
DEFINE_bool(backward_grid_save_orthomosaic_jpg, true, "");
DEFINE_string(backward_grid_orthomosaic_jpg_filename, "", "");
DEFINE_double(backward_grid_orthomosaic_elevation_m, 0.0, "");
DEFINE_bool(backward_grid_use_digital_elevation_map, true, "");
DEFINE_bool(backward_grid_grid_mode_batch, true, "");


int main(int  argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  // TODO(hitimo): Remove ROS dependency here.
  ros::init(argc, argv, "ortho_backward_grid");
  ros::Time::init();

  // Parse input parameters.
  const std::string& base = FLAGS_backward_grid_data_directory;
  const std::string& filename_camera_rig =
      FLAGS_backward_grid_filename_camera_rig;
  const std::string& filename_poses = FLAGS_backward_grid_filename_poses;
  const std::string& filename_images = base + FLAGS_backward_grid_prefix_images;

  ortho::SettingsGrid settings;
  settings.orthomosaic_easting_min =
      FLAGS_backward_grid_orthomosaic_easting_min;
  settings.orthomosaic_easting_max =
      FLAGS_backward_grid_orthomosaic_easting_max;
  settings.orthomosaic_northing_min =
      FLAGS_backward_grid_orthomosaic_northing_min;
  settings.orthomosaic_northing_max =
      FLAGS_backward_grid_orthomosaic_northing_max;
  settings.orthomosaic_resolution =
      FLAGS_backward_grid_orthomosaic_resolution;
  settings.show_orthomosaic_opencv =
      FLAGS_backward_grid_show_orthomosaic_opencv;
  settings.save_orthomosaic_jpg =
      FLAGS_backward_grid_save_orthomosaic_jpg;
  settings.orthomosaic_jpg_filename =
      FLAGS_backward_grid_orthomosaic_jpg_filename;
  settings.orthomosaic_elevation_m =
      FLAGS_backward_grid_orthomosaic_elevation_m;
  settings.use_digital_elevation_map =
      FLAGS_backward_grid_use_digital_elevation_map;
  if (FLAGS_backward_grid_grid_mode_batch) {
    settings.mode = ortho::Mode::Batch;
  } else {
    settings.mode = ortho::Mode::Incremental;
  }

  // Load camera rig from file.
  io::AerialMapperIO io_handler;
  const std::string& filename_camera_rig_yaml = base + filename_camera_rig;
  std::shared_ptr<aslam::NCamera> ncameras =
      io_handler.loadCameraRigFromFile(filename_camera_rig_yaml);
  CHECK(ncameras);

  // Load body poses from file.
  Poses T_G_Bs;
  const std::string& path_filename_poses = base + filename_poses;
  io::PoseFormat pose_format = io::PoseFormat::Standard;
  io_handler.loadPosesFromFile(pose_format, path_filename_poses, &T_G_Bs);

  // Load images from file.
  size_t num_poses = T_G_Bs.size();
  Images images;
  io_handler.loadImagesFromFile(filename_images, num_poses, &images);

  // Construct the orthomosaic by back-projecting cell center into image
  // and querring pixel intensity in image.
  ortho::OrthoBackwardGrid mosaic(ncameras, T_G_Bs, images, settings);
  return 0;
}
