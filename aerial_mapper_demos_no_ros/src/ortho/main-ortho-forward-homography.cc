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

DEFINE_string(data_directory, "", "");
DEFINE_string(filename_poses, "", "");
DEFINE_string(prefix_images, "", "");
DEFINE_string(filename_camera_rig, "", "");
DEFINE_double(origin_easting_m, 0.0, "");
DEFINE_double(origin_northing_m, 0.0, "");
DEFINE_double(origin_elevation_m, 0.0, "");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  // TODO(hitimo): Remove ROS dependency here.
  ros::init(argc, argv, "ortho_forward_homography");

  // Parse input parameters.
  const std::string& base = FLAGS_data_directory;
  const std::string& filename_camera_rig = FLAGS_filename_camera_rig;
  const std::string& filename_poses = FLAGS_filename_poses;
  const std::string& filename_images = base + FLAGS_prefix_images;
  const Eigen::Vector3d origin(FLAGS_origin_easting_m,
                               FLAGS_origin_northing_m,
                               FLAGS_origin_elevation_m);

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
  enum Mode { Incremental, Batch };
  Mode mode;
  mode = Mode::Batch;
  CHECK(ncameras);
  ortho::OrthoForwardHomography mosaic(ncameras, origin);
  if (mode == Mode::Incremental) {
    for (size_t i = 0u; i < images.size(); ++i) {
      LOG(INFO) << i << "/" << images.size();
      CHECK(i < images.size());
      const Image& image = images[i];
      CHECK(i < T_G_Bs.size());
      const Pose& T_G_B = T_G_Bs[i];
      mosaic.updateOrthomosaic(T_G_B, image);
    }
  } else if (mode == Mode::Batch) {
    mosaic.batch(T_G_Bs, images);
  }

  return 0;
}
