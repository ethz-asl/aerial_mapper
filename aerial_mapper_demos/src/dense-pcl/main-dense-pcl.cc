/*
 *    Filename: main-dense-pcl.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// SYSTEM
#include <string>

// NON-SYSTEM
#include <aerial-mapper-dense-pcl/dense-pcl-planar-rectification.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>

DEFINE_string(data_directory, "", "");
DEFINE_string(filename_camera_rig, "", "");
DEFINE_string(filename_poses, "", "");
DEFINE_string(prefix_images, "", "");
DEFINE_int32(dense_pcl_use_every_nth_image, 1, "");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "main_dense_pcl");

  // Parse input parameters.
  const std::string& base = FLAGS_data_directory;
  const std::string& filename_camera_rig = FLAGS_filename_camera_rig;
  const std::string& filename_poses = FLAGS_filename_poses;
  const std::string& filename_images = base + FLAGS_prefix_images;

  LOG(INFO) << "Loading camera rig from file.";
  io::AerialMapperIO io_handler;
  const std::string& filename_camera_rig_yaml = base + filename_camera_rig;
  std::shared_ptr<aslam::NCamera> ncameras =
      io_handler.loadCameraRigFromFile(filename_camera_rig_yaml);
  CHECK(ncameras);

  LOG(INFO) << "Loading body poses from file.";
  Poses T_G_Bs;
  const std::string& path_filename_poses = base + filename_poses;
  io::PoseFormat pose_format = io::PoseFormat::Standard;
  io_handler.loadPosesFromFile(pose_format, path_filename_poses, &T_G_Bs);

  LOG(INFO) << "Loading images from file.";
  size_t num_poses = T_G_Bs.size();
  Images images;
  io_handler.loadImagesFromFile(filename_images, num_poses, &images);

  LOG(INFO) << "Perform dense reconstruction using planar rectification.";
  dense_pcl::Settings settings_dense_pcl;
  settings_dense_pcl.use_every_nth_image = FLAGS_dense_pcl_use_every_nth_image;
  dense_pcl::PlanarRectification dense_reconstruction(ncameras,
                                                      settings_dense_pcl);
  AlignedType<std::vector, Eigen::Vector3d>::type point_cloud;
  dense_reconstruction.addFrames(T_G_Bs, images, &point_cloud);

  return 0;
}
