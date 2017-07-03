/*
 *    Filename: main_from_file.cc
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

DEFINE_double(origin_easting_m, 0.0, "");
DEFINE_double(origin_elevation_m, 0.0, "");
DEFINE_double(origin_northing_m, 0.0, "");
DEFINE_string(data_directory, "", "");
DEFINE_string(filename_camera_rig, "", "");
DEFINE_string(filename_poses, "", "");
DEFINE_string(prefix_images, "", "");
DEFINE_int32(use_every_nth_image, 1, "");


int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  // TODO(hitimo): Remove ROS dependency here.
  ros::init(argc, argv, "dense_pcl");

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
  io_handler.subtractOriginFromPoses(origin, &T_G_Bs);

  // Load images from file.
  size_t num_poses = T_G_Bs.size();
  Images images;
  io_handler.loadImagesFromFile(filename_images, num_poses, &images);

  // Perform dense reconstruction using planar rectification.
  dense_pcl::Settings settings;
  dense_pcl::PlanarRectification
      online_planar_rectification(ncameras, settings);
  size_t skip = 0u;
  for (size_t i = 0u; i < images.size(); ++i) {
    LOG(INFO) << i << "/" << images.size();
    cv::Mat image = images[i];
    if (++skip % FLAGS_use_every_nth_image == 0) {
      LOG(INFO) << "i = " << i;
      const Pose& T_G_B = T_G_Bs[i];
      online_planar_rectification.addFrame(T_G_B, image);
      cv::imshow("Image", image);
      cv::waitKey(0);
    }
  }
  LOG(INFO) << "Finished!";

  return 0;
}
