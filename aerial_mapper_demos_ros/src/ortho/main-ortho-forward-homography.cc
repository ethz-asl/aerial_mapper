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

DEFINE_string(
    data_directory,
    "/media/timo/scnd/catkin_ws_aerial_mapper/src/aerial_mapper/data/", "");
DEFINE_string(filename_poses, "opt_poses.txt", "");
DEFINE_string(prefix_images, "opt_image", "");
DEFINE_string(filename_camera_rig, "camera_fixed_wing.yaml", "");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "ortho_forward_homography");

  const std::string& base = FLAGS_data_directory;
  const std::string& filename_camera_rig = FLAGS_filename_camera_rig;

  // Load camera rig from file.
  io::AerialMapperIO io_handler;
  std::shared_ptr<aslam::NCamera> ncameras;
  const std::string& filename_camera_rig_yaml = base + filename_camera_rig;
  io_handler.loadCameraRigFromFile(filename_camera_rig_yaml, ncameras);
  const Eigen::Vector3d origin(464980, 5.27226e+06, 414.087);
  ortho::OrthoForwardHomography mosaic(ncameras, origin);

  // Load poses from file.
  Poses T_G_Bs;
  const std::string& filename_poses = base + filename_poses;
  io::PoseFormat pose_format = io::PoseFormat::Standard;
  io_handler.loadPosesFromFile(pose_format, filename_poses, &T_G_Bs);

  // Load images from file.
  const std::string& filename_images = base + FLAGS_prefix_images;
  size_t num_poses = T_G_Bs.size();
  Images images;
  io_handler.loadImagesFromFile(filename_images, num_poses, &images);

  enum Mode { Incremental, Batch };
  Mode mode;
  mode = Mode::Batch;

  // Construct the mosaic by computing the homography that projects
  // the image onto the ground plane.
  if (mode == Mode::Incremental) {
    for (size_t i = 0u; i < images.size(); ++i) {
      LOG(INFO) << i << "/" << images.size();
      const Image& image = images[i];
      const Pose& T_G_B = T_G_Bs[i];
      mosaic.updateOrthomosaic(T_G_B, image);
    }
  } else if (mode == Mode::Batch) {
    mosaic.batch(T_G_Bs, images);
  }

  LOG(INFO) << "Finished!";

  return 0;
}
