/*
 *    Filename: main-ortho-backward-grid.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// SYSTEM
#include <memory>

// NON-SYSTEM
#include <aerial-mapper-dense-pcl/dense-pcl-planar-rectification.h>
#include <aerial-mapper-dsm/dsm.h>
#include <aerial-mapper-grid-map/aerial-mapper-grid-map.h>
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
DEFINE_bool(backward_grid_use_grid_map, true, "");
DEFINE_bool(load_point_cloud_from_file, false, "");
DEFINE_string(point_cloud_filename, "", "");
DEFINE_int32(dense_pcl_use_every_nth_image, 10, "");

void parseSettingsOrtho(ortho::Settings* settings_ortho);

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "ortho_backward_grid");
  ros::Time::init();

  // Parse input parameters.
  const std::string& base = FLAGS_backward_grid_data_directory;
  const std::string& filename_camera_rig =
      FLAGS_backward_grid_filename_camera_rig;
  const std::string& filename_poses = FLAGS_backward_grid_filename_poses;
  const std::string& filename_images = base + FLAGS_backward_grid_prefix_images;

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

  // Retrieve dense point cloud.
  Aligned<std::vector, Eigen::Vector3d>::type point_cloud;
  if (FLAGS_load_point_cloud_from_file) {
    // Either load point cloud from file..
    CHECK(!FLAGS_point_cloud_filename.empty());
    io_handler.loadPointCloudFromFile(FLAGS_point_cloud_filename, &point_cloud);
  } else {
    // .. or generate via dense reconstruction from poses and images.
    dense_pcl::Settings settings_dense_pcl;
    settings_dense_pcl.use_every_nth_image =
        FLAGS_dense_pcl_use_every_nth_image;
    dense_pcl::PlanarRectification dense_reconstruction(ncameras,
                                                        settings_dense_pcl);
    dense_reconstruction.addFrames(T_G_Bs, images, &point_cloud);
  }

  LOG(INFO) << "Initialize layered map.";
  grid_map::Settings settings_aerial_grid_map;
  settings_aerial_grid_map.center_easting = 0.0;
  settings_aerial_grid_map.center_northing = 0.0;
  settings_aerial_grid_map.delta_easting = 200.0;
  settings_aerial_grid_map.delta_northing = 200.0;
  settings_aerial_grid_map.resolution = 1.0;
  grid_map::AerialGridMap map(settings_aerial_grid_map);

  LOG(INFO) << "Create DSM (batch).";
  dsm::Settings settings_dsm;
  settings_dsm.center_easting = settings_aerial_grid_map.center_easting;
  settings_dsm.center_northing = settings_aerial_grid_map.center_northing;
  dsm::Dsm digital_surface_map(settings_dsm);
  digital_surface_map.initializeAndFillKdTree(point_cloud);
  digital_surface_map.updateElevationLayer(map.getMutable());

  LOG(INFO) << "Construct the orthomosaic (batch).";
  ortho::Settings settings_ortho;
  parseSettingsOrtho(&settings_ortho);
  ortho::OrthoBackwardGrid mosaic(ncameras, settings_ortho, map.getMutable());
  // Orthomosaic via back-projecting cell center into image
  // and quering pixel intensity in image.
  mosaic.process(T_G_Bs, images, map.getMutable());

  LOG(INFO) << "Publish until shutdown.";
  map.publishUntilShutdown();

  return 0;
}

void parseSettingsOrtho(ortho::Settings* settings_ortho) {
  settings_ortho->orthomosaic_easting_min =
      FLAGS_backward_grid_orthomosaic_easting_min;
  settings_ortho->orthomosaic_easting_max =
      FLAGS_backward_grid_orthomosaic_easting_max;
  settings_ortho->orthomosaic_northing_min =
      FLAGS_backward_grid_orthomosaic_northing_min;
  settings_ortho->orthomosaic_northing_max =
      FLAGS_backward_grid_orthomosaic_northing_max;
  settings_ortho->orthomosaic_resolution =
      FLAGS_backward_grid_orthomosaic_resolution;
  settings_ortho->show_orthomosaic_opencv =
      FLAGS_backward_grid_show_orthomosaic_opencv;
  settings_ortho->save_orthomosaic_jpg =
      FLAGS_backward_grid_save_orthomosaic_jpg;
  settings_ortho->orthomosaic_jpg_filename =
      FLAGS_backward_grid_orthomosaic_jpg_filename;
  settings_ortho->orthomosaic_elevation_m =
      FLAGS_backward_grid_orthomosaic_elevation_m;
  settings_ortho->use_digital_elevation_map =
      FLAGS_backward_grid_use_digital_elevation_map;
}
