/*
 *    Filename: main-ortho-forward-homography.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// SYSTEM
#include <memory>

// NON-SYSTEM
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-ortho/ortho-forward-homography.h>
#include <gflags/gflags.h>

DEFINE_string(forward_homography_data_directory, "",
              "Directory to poses, images, and calibration file.");
DEFINE_string(forward_homography_filename_poses, "",
              "Name of the file that contains positions and orientations for "
              "every camera in the global/world frame, i.e. T_G_B");
DEFINE_string(forward_homography_prefix_images, "",
              "Prefix of the images to be loaded, e.g. 'images_'");
DEFINE_string(
    forward_homography_filename_camera_rig, "",
    "Name of the camera calibration file (intrinsics). File ending: .yaml");
DEFINE_double(forward_homography_origin_easting_m, 0.0,
              "Origin [m] of the homography-based orthomosaic (easting).");
DEFINE_double(forward_homography_origin_northing_m, 0.0,
              "Origin [m] of the homography-based orthomosaic (northing).");
DEFINE_double(forward_homography_origin_elevation_m, 0.0,
              "Origin [m] of the homography-based orthomosaic (elevation).");
DEFINE_double(
    forward_homography_ground_plane_elevation_m, 414.0,
    "Elevation [m] of the orthomosaic (flat/ground plane assumption).");
DEFINE_int32(forward_homography_width_mosaic_pixels, 1000,
             "Width of orthomosaic in pixels.");
DEFINE_int32(forward_homography_height_mosaic_pixels, 1000,
             "Heigh of the orthomosaic in pixels");
DEFINE_bool(forward_homography_batch, true,
            "Use batch? Otherwise process images incrementally.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "ortho_forward_homography");

  // Parse input parameters.
  const std::string& base = FLAGS_forward_homography_data_directory;
  const std::string& filename_camera_rig =
      FLAGS_forward_homography_filename_camera_rig;
  const std::string& filename_poses = FLAGS_forward_homography_filename_poses;
  const std::string& filename_images =
      base + FLAGS_forward_homography_prefix_images;
  const Eigen::Vector3d origin(FLAGS_forward_homography_origin_easting_m,
                               FLAGS_forward_homography_origin_northing_m,
                               FLAGS_forward_homography_origin_elevation_m);

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

  // Construct the mosaic by computing the homography that projects
  // the image onto the ground plane.
  ortho::Settings settings_ortho;
  settings_ortho.ground_plane_elevation_m =
      FLAGS_forward_homography_ground_plane_elevation_m;
  settings_ortho.height_mosaic_pixels =
      FLAGS_forward_homography_height_mosaic_pixels;
  settings_ortho.width_mosaic_pixels =
      FLAGS_forward_homography_width_mosaic_pixels;
  settings_ortho.origin = origin;

  CHECK(ncameras);
  ortho::OrthoForwardHomography mosaic(ncameras, settings_ortho);
  if (!FLAGS_forward_homography_batch) {
    for (size_t i = 0u; i < images.size(); ++i) {
      LOG(INFO) << i << "/" << images.size();
      CHECK(i < images.size());
      const Image& image = images[i];
      CHECK(i < T_G_Bs.size());
      const Pose& T_G_B = T_G_Bs[i];
      mosaic.updateOrthomosaic(T_G_B, image);
    }
  } else {
    mosaic.batch(T_G_Bs, images);
  }

  return 0;
}
