#include "fw_online_digital_elevation_map_node/fw-digital-elevation-map.h"

#include <memory>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

DEFINE_string(DEM_filename_xyz, "/media/timo/scnd/experiments/dem_test/cropped.xyz", "filename pointcloud");
DEFINE_string(DEM_output_folder, "/tmp/", "Result output folder.");
DEFINE_double(DEM_interpolation_radius, 5.0, "Radius of the interpolation (KD-tree)");
DEFINE_int32(DEM_color_palette, 6, "Color palette enum");
DEFINE_double(DEM_resolution, 1, "Resolution of the Digital elevation map.");
DEFINE_int32(DEM_UTM_code, 33, "UTM code used for the geotiff.");
DEFINE_bool(DEM_show_output, true, "Show the output as opencv image?");
DEFINE_bool(DEM_save_cv_mat_height_map, true, "");
DEFINE_double(DEM_easting_min, 464899.00, "");
DEFINE_double(DEM_northing_min, 5272160.00, "");
DEFINE_double(DEM_easting_max, 465056.47, "");
DEFINE_double(DEM_northing_max, 5272349.53, "");

bool got_point_cloud;
std::unique_ptr<FwOnlineDigitalElevationMap> dem_;

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  if (got_point_cloud) {
    return;
  }
  VLOG(1) << "Received point-cloud.";
  got_point_cloud = true;
  //const Eigen::Vector3d origin(464980, 5.27226e+06, 414.087);
  const Eigen::Vector3d origin(200.0, 200.0, 0.0);
  dem_.reset(new FwOnlineDigitalElevationMap(cloud_msg, origin));
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  got_point_cloud = false;
  ros::init(argc, argv, "online_digital_elevation_map_node");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  sub = nh.subscribe ("/planar_rectification/point_cloud", 1, cloud_callback);
  ros::spin();

  return 0;
}
