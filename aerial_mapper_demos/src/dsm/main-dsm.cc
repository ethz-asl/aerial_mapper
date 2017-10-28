/*
 *    Filename: main-dsm.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */


// NON-SYSTEM
#include <aerial-mapper-dense-pcl/dense-pcl-planar-rectification.h>
#include <aerial-mapper-dsm/dsm.h>
#include <aerial-mapper-grid-map/aerial-mapper-grid-map.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

DEFINE_string(data_directory, "", "");
DEFINE_string(filename_camera_rig, "", "");
DEFINE_string(filename_poses, "", "");
DEFINE_string(prefix_images, "", "");
DEFINE_string(filename_point_cloud, "", "");
DEFINE_int32(dense_pcl_use_every_nth_image, 10, "");
DEFINE_double(center_easting, 0.0, "");
DEFINE_double(center_northing, 0.0, "");
DEFINE_double(delta_easting, 0.0, "");
DEFINE_double(delta_northing, 0.0, "");
DEFINE_double(resolution, 1.0, "");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "dsm_from_file");

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
  Aligned<std::vector, Eigen::Vector3d>::type point_cloud;
  if (!FLAGS_filename_point_cloud.empty()) {
    // Either load point cloud from file..
    io_handler.loadPointCloudFromFile(FLAGS_filename_point_cloud, &point_cloud);
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
  settings_aerial_grid_map.center_easting = FLAGS_center_easting;
  settings_aerial_grid_map.center_northing = FLAGS_center_northing;
  settings_aerial_grid_map.delta_easting = FLAGS_delta_easting;
  settings_aerial_grid_map.delta_northing = FLAGS_delta_northing;
  settings_aerial_grid_map.resolution = FLAGS_resolution;
  grid_map::AerialGridMap map(settings_aerial_grid_map);

  LOG(INFO) << "Create DSM (batch).";
  dsm::Settings settings_dsm;
  settings_dsm.center_easting = settings_aerial_grid_map.center_easting;
  settings_dsm.center_northing = settings_aerial_grid_map.center_northing;
  dsm::Dsm digital_surface_map(settings_dsm);
  digital_surface_map.process(point_cloud, map.getMutable());

  LOG(INFO) << "Publish until shutdown.";
  map.publishUntilShutdown();

  return 0;
}
