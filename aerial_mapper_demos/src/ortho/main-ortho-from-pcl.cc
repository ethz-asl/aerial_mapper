/*
 *    Filename: main-ortho-from-pcl.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// NON-SYSTEM
#include <aerial-mapper-dense-pcl/stereo.h>
#include <aerial-mapper-grid-map/aerial-mapper-grid-map.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-ortho/ortho-from-pcl.h>
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

DEFINE_bool(ortho_from_pcl_show_orthomosaic_opencv, true,
            "Show the orthomosaic using opencv?");
DEFINE_bool(ortho_from_pcl_use_adaptive_interpolation, false,
            "Use adaptive interpolation in the orthomosaic generation? Used to "
            "handle missing data.");
DEFINE_int32(ortho_from_pcl_interpolation_radius, 10,
             "Interpolation radius [m].");
DEFINE_double(ortho_from_pcl_center_easting, 0.0,
              "Center [m] of the grid_map (easting).");
DEFINE_double(ortho_from_pcl_center_northing, 0.0,
              "Center [m] of the grid_map (northing).");
DEFINE_double(ortho_from_pcl_delta_easting, 100.0,
              "Width [m] of the grid_map, starting from center.");
DEFINE_double(ortho_from_pcl_delta_northing, 100.0,
              "Height [m] of the grid_map, starting from center");
DEFINE_double(ortho_from_pcl_resolution, 1.0,
              "Resolution of the grid_map [m].");
DEFINE_string(data_directory, "",
              "Directory to poses, images, and calibration file.");
DEFINE_string(ortho_from_pcl_orthomosaic_jpg_filename, "",
              "Name of the output image storing the orthomsaic.");
DEFINE_string(filename_poses, "",
              "Name of the file that contains positions and orientations for "
              "every camera in the global/world frame, i.e. T_G_B");
DEFINE_string(prefix_images, "",
              "Prefix of the images to be loaded, e.g. 'images_'");
DEFINE_string(
    filename_camera_rig, "",
    "Name of the camera calibration file (intrinsics). File ending: .yaml");
DEFINE_bool(load_point_cloud_from_file, false,
            "Load point cloud from file? Otherwise generate the point cloud "
            "from the provided images, camera poses, camera intrinsicspoint "
            "cloud from images.");
DEFINE_string(filename_point_cloud, "",
              "Name of the file that contains the point cloud.");
DEFINE_int32(dense_pcl_use_every_nth_image, 10,
             "Only use every n-th image in the densification process.");
DEFINE_bool(use_BM, true,
            "Use BM Blockmatching if true. Use SGBM (=Semi-Global-) "
            "Blockmatching if false.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "ortho_from_pcl");
  ros::NodeHandle nh;

  // Parse input parameters.
  const std::string& base = FLAGS_data_directory;
  const std::string& filename_camera_rig = FLAGS_filename_camera_rig;
  const std::string& filename_poses = FLAGS_filename_poses;
  const std::string& filename_images = base + FLAGS_prefix_images;

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
  AlignedType<std::vector, Eigen::Vector3d>::type point_cloud;
  std::vector<int> point_cloud_intensities;
  if (FLAGS_load_point_cloud_from_file) {
    // Either load point cloud from file..
    CHECK(!FLAGS_filename_point_cloud.empty());
    io_handler.loadPointCloudFromFile(FLAGS_filename_point_cloud, &point_cloud,
                                      &point_cloud_intensities);
  } else {
    // .. or generate via dense reconstruction from poses and images.
    stereo::Settings settings_dense_pcl;
    settings_dense_pcl.use_every_nth_image =
        FLAGS_dense_pcl_use_every_nth_image;
    LOG(INFO) << "Perform dense reconstruction using planar rectification.";
    stereo::BlockMatchingParameters block_matching_params;
    block_matching_params.use_BM = FLAGS_use_BM;
    stereo::Stereo stereo(ncameras, settings_dense_pcl, block_matching_params);
    stereo.addFrames(T_G_Bs, images, &point_cloud, &point_cloud_intensities);
  }

  LOG(INFO) << "Initialize layered map.";
  grid_map::Settings settings_aerial_grid_map;
  settings_aerial_grid_map.center_easting = FLAGS_ortho_from_pcl_center_easting;
  settings_aerial_grid_map.center_northing =
      FLAGS_ortho_from_pcl_center_northing;
  settings_aerial_grid_map.delta_easting = FLAGS_ortho_from_pcl_delta_easting;
  settings_aerial_grid_map.delta_northing = FLAGS_ortho_from_pcl_delta_northing;
  settings_aerial_grid_map.resolution = FLAGS_ortho_from_pcl_resolution;
  grid_map::AerialGridMap map(settings_aerial_grid_map);

  // Orthomosaic from point cloud.
  ortho::Settings settings;
  settings.interpolation_radius = FLAGS_ortho_from_pcl_interpolation_radius;
  settings.use_adaptive_interpolation =
      FLAGS_ortho_from_pcl_use_adaptive_interpolation;
  settings.show_orthomosaic_opencv =
      FLAGS_ortho_from_pcl_show_orthomosaic_opencv;
  settings.orthomosaic_jpg_filename =
      FLAGS_ortho_from_pcl_orthomosaic_jpg_filename;
  CHECK(point_cloud.size() > 0);
  CHECK(point_cloud.size() == point_cloud_intensities.size());

  // Generate the orthomosaic from the point cloud.
  ortho::OrthoFromPcl mosaic(settings);
  mosaic.process(point_cloud, point_cloud_intensities, map.getMutable());

  LOG(INFO) << "Publish until shutdown.";
  map.publishUntilShutdown();

  return 0;
}
